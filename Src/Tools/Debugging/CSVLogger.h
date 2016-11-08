#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <list>

#ifdef RELEASE
#undef LOGGING
#endif

// Use this to globally disable logging
// #undef LOGGING

/*

 Other method to disable all loggings:
 Search and replace by using this regular Expression:

 \//#define LOGGING

 reaplaced by:

 //#define LOGGING

 */

#ifndef LOGGING
#define LOG(titel, name, data) /**/
#define MARK(titel, name) /**/
#define FLUSH /**/
#else
#define LOG(titel, name, data) CSVLogger::log(titel, name, data)
#define MARK(titel, name) CSVLogger::mark(titel, name)
#define FLUSH	CSVLogger::flush();
#endif

class CSVLogger
{
public:
	CSVLogger(void);
	~CSVLogger(void);

  static void log(std::string titel, std::string name, std::string data);
	static void log(std::string titel, std::string name, unsigned int data);
	static void log(std::string titel, std::string name, double data);
	static void log(std::string titel, std::string name, unsigned long data);
	static void log(std::string titel, std::string name, long data);
	static void log(std::string titel, std::string name, int data);
	static void mark(std::string titel, std::string name);
	static void flush();
private:
	struct Column
	{
		std::string name;
		std::string data;
		bool filled;
	};

	typedef std::list<Column *> ColumnList;

	struct Logfile
	{
		std::ofstream s;
		std::string name;
		ColumnList columns;
		bool headerWritten;
		ColumnList::iterator lastFound;
	};

	typedef std::list<Logfile *> LogfileList;

	static LogfileList logs;

	static bool add(std::string titel, std::string name, std::string data);

	static Logfile *getFile(std::string name);
	static Column *getColumn(Logfile *f, std::string name);
	static Logfile *addFile(std::string name);
	static Column *addColumn(Logfile *f, std::string name);
	static void flush(Logfile *f);
	static bool disabled;
};
