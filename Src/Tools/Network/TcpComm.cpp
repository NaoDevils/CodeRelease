/**
 * @file Platform/Common/TcpComm.cpp
 *
 * Implementation of class TcpComm.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "TcpComm.h"
#include "Platform/BHAssert.h"

#include <cerrno>
#include <fcntl.h>

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#ifdef WINDOWS
#define socket_t SOCKET
#else
#define socket_t int
#endif

#ifdef WINDOWS
#define ERRNO WSAGetLastError()
#define RESET_ERRNO WSASetLastError(0)
#define NON_BLOCK(socket) ioctlsocket(socket, FIONBIO, (u_long*)"NONE")
#define CLOSE(socket) closesocket(socket)
#undef EWOULDBLOCK
#define EWOULDBLOCK WSAEWOULDBLOCK
#undef EINPROGRESS
#define EINPROGRESS WSAEINPROGRESS

struct _WSAFramework
{
  _WSAFramework()
  {
    WORD wVersionRequested = MAKEWORD(2, 2);
    WSADATA wsaData;
    WSAStartup(wVersionRequested, &wsaData);
  }
  ~_WSAFramework() { WSACleanup(); }
} _wsaFramework;

#else

#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define ERRNO errno
#define RESET_ERRNO errno = 0
#define NON_BLOCK(socket) fcntl(socket, F_SETFL, O_NONBLOCK)
#define CLOSE(socket) ::close(socket)

#endif

#ifndef LINUX
#define MSG_NOSIGNAL 0
#endif

struct TcpComm::Pimpl
{
  socket_t createSocket = 0; /**< The handle of the basic socket. */
  socket_t transferSocket = 0; /**< The handle of the actual transfer socket. */
  sockaddr_in address; /**< The socket data->address. */
};

TcpComm::TcpComm(const char* ip, int port, int maxPackageSendSize, int maxPackageReceiveSize)
    : data(std::make_unique<Pimpl>()), maxPackageSendSize(maxPackageSendSize), maxPackageReceiveSize(maxPackageReceiveSize)
{
  data->address.sin_family = AF_INET;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  data->address.sin_port = htons(static_cast<unsigned short>(port));
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  if (ip) // connect as client?
    data->address.sin_addr.s_addr = inet_addr(ip);
  else
  {
    data->createSocket = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT(data->createSocket > 0);
    int val = 1;
    setsockopt(data->createSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));
    data->address.sin_addr.s_addr = INADDR_ANY;
    VERIFY(bind(data->createSocket, (sockaddr*)&data->address, sizeof(sockaddr_in)) == 0);
    VERIFY(listen(data->createSocket, SOMAXCONN) == 0);
    NON_BLOCK(data->createSocket);
  }
  checkConnection();
}

TcpComm::~TcpComm()
{
  this->close();
}

void TcpComm::close()
{
  if (connected())
    closeTransferSocket();
  if (data->createSocket > 0)
    CLOSE(data->createSocket);
}

TcpComm::TcpComm(TcpComm&& other) noexcept
    : data(std::make_unique<Pimpl>(*other.data)), overallBytesSent(other.overallBytesSent), overallBytesReceived(other.overallBytesReceived),
      maxPackageSendSize(other.maxPackageSendSize), maxPackageReceiveSize(other.maxPackageReceiveSize), wasConnected(other.wasConnected)
{
  other.data->createSocket = 0;
  other.data->transferSocket = 0;
}

TcpComm& TcpComm::operator=(TcpComm&& other) noexcept
{
  this->close();
  this->data->createSocket = other.data->createSocket;
  this->data->transferSocket = other.data->transferSocket;
  this->data->address = other.data->address;
  this->overallBytesSent = other.overallBytesSent;
  this->overallBytesReceived = other.overallBytesReceived;
  this->maxPackageSendSize = other.maxPackageSendSize;
  this->maxPackageReceiveSize = other.maxPackageReceiveSize;
  this->wasConnected = other.wasConnected;
  other.data->createSocket = 0;
  other.data->transferSocket = 0;
  return *this;
}

bool TcpComm::checkConnection()
{
  if (!connected())
  {
    if (data->createSocket)
      data->transferSocket = accept(data->createSocket, nullptr, nullptr);
    else if (!wasConnected)
    {
      data->transferSocket = socket(AF_INET, SOCK_STREAM, 0);
      ASSERT(connected());
      if (connect(data->transferSocket, (sockaddr*)&data->address, sizeof(sockaddr_in)) != 0)
      {
        CLOSE(data->transferSocket);
        data->transferSocket = 0;
      }
    }

    if (connected())
    {
      wasConnected = true;
      NON_BLOCK(data->transferSocket); // switch socket to nonblocking
#ifdef MACOS
      int yes = 1;
      VERIFY(!setsockopt(data->transferSocket, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes)));
#endif
      if (maxPackageSendSize)
        VERIFY(!setsockopt(data->transferSocket, SOL_SOCKET, SO_SNDBUF, (char*)&maxPackageSendSize, sizeof(maxPackageSendSize)));
      if (maxPackageReceiveSize)
        VERIFY(!setsockopt(data->transferSocket, SOL_SOCKET, SO_RCVBUF, (char*)&maxPackageReceiveSize, sizeof(maxPackageReceiveSize)));
      return true;
    }
    else
      return false;
  }
  else
    return true;
}

void TcpComm::closeTransferSocket()
{
  CLOSE(data->transferSocket);
  data->transferSocket = 0;
}

bool TcpComm::receive(unsigned char* buffer, int size, bool wait)
{
  if (!checkConnection())
    return false;

  if (!wait)
  {
    RESET_ERRNO;
#ifdef WINDOWS
    char c;
    int received = recv(data->transferSocket, &c, 1, MSG_PEEK);
    if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS) || ioctlsocket(data->transferSocket, FIONREAD, (u_long*)&received) != 0)
    {
      closeTransferSocket();
      return false;
    }
    else if (received == 0)
      return false;
#else
    int received = (int)recv(data->transferSocket, (char*)buffer, size, MSG_PEEK);
    if (received < size)
    {
      if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS))
        closeTransferSocket();
      return false;
    }
#endif
  }

  int received = 0;
  while (received < size)
  {
    RESET_ERRNO;

    int received2 = (int)recv(data->transferSocket, (char*)buffer + received, size - received, 0);

    if (!received2 || (received2 < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) // error during reading of package
    {
      closeTransferSocket();
      return false;
    }
    else if (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS) // wait for the rest
    {
      received2 = 0;
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set rset;
      FD_ZERO(&rset);
      FD_SET(data->transferSocket, &rset);
      if (select(static_cast<int>(data->transferSocket + 1), &rset, 0, 0, &timeout) == -1)
      {
        closeTransferSocket();
        return false; // error while waiting
      }
    }
    received += received2;
    overallBytesReceived += received2;
  }
  return true; // ok, data received
}

bool TcpComm::connected() const
{
  return data->transferSocket > 0;
}

bool TcpComm::send(const unsigned char* buffer, int size)
{
  if (!checkConnection())
    return false;

  RESET_ERRNO;
  int sent = (int)::send(data->transferSocket, (const char*)buffer, size, MSG_NOSIGNAL);
  if (sent > 0)
  {
    overallBytesSent += sent;
    while (sent < size && (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS || ERRNO == 0))
    {
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set wset;
      FD_ZERO(&wset);
      FD_SET(data->transferSocket, &wset);
      RESET_ERRNO;
      if (select(static_cast<int>(data->transferSocket + 1), 0, &wset, 0, &timeout) == -1)
        break;
      RESET_ERRNO;
      int sent2 = (int)::send(data->transferSocket, (const char*)buffer + sent, size - sent, MSG_NOSIGNAL);
      if (sent2 >= 0)
      {
        sent += sent2;
        overallBytesSent += sent;
      }
    }
  }

  if (ERRNO == 0 && sent == size)
    return true;
  else
  {
    closeTransferSocket();
    return false;
  }
}
