#! /usr/bin/env python3
#-------------------------------------------------------------------------------
# Name:			ssh
# Info:			Provide a connector class which establish an ssh connection
#               to execute commands and transfer files 
# Author:		Dominik Brämer
# Created:		05.08.2019
# Version:		19-08-06
#-------------------------------------------------------------------------------
import paramiko
import os
import warnings
import socket
import time

warnings.filterwarnings(action='ignore',module='.*paramiko.*')

class Connector:

    def __init__(self, hostname: str, username: str = 'nao', password: str = 'nao'):
        self.exitStatus = 0
        self.passwd = password
        self.sshClient = paramiko.client.SSHClient()

        self.__workingDir = os.getcwd()
        open(self.__workingDir + '/host_keys', 'w').close()
        self.sshClient.load_host_keys(self.__workingDir + '/host_keys')
        self.sshClient.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            self.sshClient.connect(hostname = hostname, username = username, password = password, look_for_keys = False, timeout  = 10)
        except socket.error:
            exit('SSH error')
        except paramiko.BadHostKeyException:
            exit('Server host key could not be verified')
        except paramiko.AuthenticationException:
            exit('Authentication error')

        self.sftp = self.sshClient.open_sftp()

        try:
            self.sshClient.exec_command('echo {passwd} | sudo -S {cmd}'.format(passwd = self.passwd, cmd = 'ls -l'))
        except paramiko.SSHException:
            print('Server fails to execute the command')
        
    def __del__(self):
        try:
            os.remove(self.__workingDir + '/host_keys')
        except:
            print('Temporary hostkey directory already removed')

        try:
            self.sshClient.close()
        except:
            print('Unable to close ssh connection')
            
        try:
            self.sftp.close()
        except:
            print('Unable to close stfp connection')

    def exec(self, cmd: str, root: bool = False, onlyFirstLine: bool = False, rebootFlag: bool = False) -> str:
        """
        Executes a command via ssh.
        If rebootFlag is not true the exitStatus of the command is stored under exitStatus of the ssh object.
        
        Args:
            cmd (str): Command to execute.
            root (bool, optional): Execute command as user root be aware the command is than not supporting shell pipes. Defaults to False.
            onlyFirstLine (bool, optional): Return only the first line of the stdout. Defaults to False.
            rebootFlag (bool, optional): Workaround flag that disable blocking and makes it possible to restart or shutdown without stuck in an open ssh connection. Defaults to False.
        
        Returns:
            str: The stdout the command or the stderr if something went wrong.
        """
        try:
            if not root:
                _, stdout, stderr = self.sshClient.exec_command(cmd, timeout=10)
            else:
                _, stdout, stderr = self.sshClient.exec_command('echo {passwd} | sudo -S {cmd}'.format(passwd = self.passwd, cmd = cmd), timeout=10)
        except paramiko.SSHException:
            print('Server fails to execute the command')
        
        if rebootFlag:
            time.sleep(3)
            return ''
        else:
            self.exitStatus = stdout.channel.recv_exit_status()

        if self.exitStatus != 0:
            if onlyFirstLine:
                return stderr.readlines()[0]
            else:
                return ''.join(stderr.readlines())
        else:
            if onlyFirstLine:
                return stdout.readlines()[0]
            else:
                return ''.join(stdout.readlines())

    def xfer(self, source: str, target: str):
        """
        Transfer a file to the remote host.
        
        Args:
            source (str): Path to the local source file.
            target (str): Path to the remote target.
        """

        self.sftp.put(source, target)