/**
 * This file declares a simple backend for the stream classes that work on
 * the "ConfigMap" format. It parses the following format and provides the data
 * as a simple syntax tree:
 *
 * map ::= record
 * record ::= field ';' { field ';' }
 * field ::= literal '=' ( literal | '{' record '}' | array )
 * array ::= '[' [ ( literal | '{' record '}' ) { ',' ( literal | '{' record '}' ) } [ ',' ] ']'
 * literal ::= '"' { anychar1 } '"' | { anychar2 }
 *
 * anychar1 must escape doublequotes and backslash with a backslash
 * anychar2 cannot contain whitespace and other characters used by the grammar.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "InOut.h"
#include "Tools/Enum.h"

#include <string>
#include <map>
#include <vector>
#include "AutoStreamable.h"

/**
 * The class is simple backend for the stream classes that work on
 * the "ConfigMap" format.
 */
class SimpleMap : public Streamable
{
public:
  /**< Base class for syntax tree nodes. */
  class Value
  {
  public:
    virtual ~Value() = default;
    virtual bool operator==(const Value&) const = 0;
    inline bool operator!=(const Value& that) const { return !(*this == that); }
  };

  /** A class representing a literal. */
  class Literal : public Value
  {
  private:
    std::string literal; /**< The literal. */
    mutable In* stream = nullptr; /**< A stream that can parse the literal. */

  public:
    Literal(const std::string& literal) : literal(literal) {}

    ~Literal()
    {
      if (stream != nullptr)
        delete stream;
    }

    operator In&() const; /**< Returns a stream that can parse the literal. */
    virtual bool operator==(const Value&) const;
  };

  /** A class representing a record of attributes, i.e. a mapping of names to values. */
  class Record : public Value, public std::map<std::string, Value*>
  {
  public:
    ~Record();
    virtual bool operator==(const Value&) const;
    Record& operator-=(const Record&);
  };

  /**< A class representing an array of values, i.e. a mapping of indices to values. */
  class Array : public Value, public std::vector<Value*>
  {
  public:
    ~Array();
    virtual bool operator==(const Value&) const;
  };

  /**
   * Construtor. Parses the stream.
   * @param stream The stream that is parsed according to the grammar given above.
   * @param name The name of the file if the stream is a file. Used for error messages.
   */
  SimpleMap(In& stream, const std::string& name = "");

  /**
   * Destructor.
   */
  ~SimpleMap();

  /**
   * Parse another stream
   * @param stream The stream that is parsed according to the grammar given above.
   * @param name The name of the file if the stream is a file. Used for error messages.
   */
  void parse(In& stream, const std::string& name = "");

  operator const Value*() const { return root; } /**< Returns the root of the syntax tree. 0 if parsing failed. */

  virtual void serialize(In*, Out*);

  SimpleMap& operator-=(const SimpleMap& that);

private:
  /** Lexicographical symbols. */
  ENUM(Symbol,
    literal, equals,
    comma, semicolon,
    lBracket, rBracket,
    lBrace, rBrace,
    eof
  );

  In* stream; /**< The stream from which is read. */
  char c; /**< The current character. 0 means EOF reached. */
  int row; /**< The current row in the stream. */
  int column; /**< The current column in the stream. */
  Symbol symbol; /**< The current lexicographical symbol. */
  std::string string; /**< The string if the current symbol is "literal". */
  Value* root; /**< The root of the syntax tree. 0 if parsing failed. */

  void nextChar(); /**< Read the next character into "c". */
  void nextSymbol(); /**< Read the next symbol into "symbol". */
  void unexpectedSymbol(); /**< Throw an exception for an unexpected symbol. */

  /**
   * Read next symbol and check whether it is the expected one.
   * Throw an exception if not.
   * @param expected The symbol expected.
   */
  void expectSymbol(Symbol expected);
  void parseRecord(Record*& r); /**< Parse a record. */
  Array* parseArray(); /**< Parse an array. */

  void serialize(In*, Out*, const SimpleMap::Value* value);
};
