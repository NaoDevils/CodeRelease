/**
 * This file implements a simple backend for the stream classes that work on
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
 * anychar2 cannot contains whitespace and other characters used by the grammar.
 *
 * @author Thomas Röfer
 */

#include "SimpleMap.h"
#include <stdexcept>
#include "InStreams.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/BHAssert.h"

SimpleMap::Literal::operator In&() const
{
  if (!stream)
    stream = new InTextMemory(literal.c_str(), literal.size());
  else
    *(InTextMemory*)stream = InTextMemory(literal.c_str(), literal.size());
  return *stream;
}

SimpleMap::Record::~Record()
{
  for (std::map<std::string, Value*>::const_iterator i = begin(); i != end(); ++i)
    delete i->second;
}

SimpleMap::Array::~Array()
{
  for (std::vector<Value*>::const_iterator i = begin(); i != end(); ++i)
    delete *i;
}

void SimpleMap::nextChar()
{
  if (c || !stream->eof())
  {
    if (c == '\n')
    {
      ++row;
      column = 1;
    }
    else if (c == '\t')
      column += 8 - (column - 1) % 8;
    else if (c != '\r')
      ++column;

    if (stream->eof())
      c = 0;
    else
      stream->read(&c, 1);
  }
}

void SimpleMap::nextSymbol()
{
  for (;;)
  {
    // Skip whitespace
    while (c && isspace(c))
      nextChar();

    string = "";
    switch (c)
    {
    case 0:
      symbol = eof;
      return; // skip nextChar()
    case '=':
      symbol = equals;
      break;
    case ',':
      symbol = comma;
      break;
    case ';':
      symbol = semicolon;
      break;
    case '[':
      symbol = lBracket;
      break;
    case ']':
      symbol = rBracket;
      break;
    case '{':
      symbol = lBrace;
      break;
    case '}':
      symbol = rBrace;
      break;
    case '"':
      string = c;
      nextChar();
      while (c && c != '"')
      {
        if (c == '\\')
        {
          string += c;
          nextChar();
        }
        if (c)
        {
          string += c;
          nextChar();
        }
      }
      if (!c)
        throw std::logic_error("Unexpected EOF in string");
      string += c;
      symbol = literal;
      break;

    case '/':
      nextChar();
      if (c == '*')
      {
        nextChar();
        char prevChar = 0;
        while (c && (c != '/' || prevChar != '*'))
        {
          prevChar = c;
          nextChar();
        }
        if (!c)
          throw std::logic_error("Unexpected EOF in comment");
        nextChar();
        continue; // jump back to skipping whitespace
      }
      else if (c == '/')
      {
        nextChar();
        while (c && c != '\n')
          nextChar();
        if (!c)
          nextChar();
        continue; // jump back to skipping whitespace
      }
      string = "/";
      // no break;

    default:
      while (c && !isspace(c) && c != '=' && c != ',' && c != ';' && c != ']' && c != '}')
      {
        string += c;
        nextChar();
      }
      symbol = literal;
      return; // skip nextChar
    }

    nextChar();
    return;
  }
}

void SimpleMap::unexpectedSymbol()
{
  if (symbol == literal)
    throw std::logic_error(std::string("Unexpected literal '") + string + "'");
  else
    throw std::logic_error(std::string("Unexpected symbol '") + getName(symbol) + "'");
}

void SimpleMap::expectSymbol(Symbol expected)
{
  if (expected != symbol)
    unexpectedSymbol();
  nextSymbol();
}

void SimpleMap::parseRecord(Record*& r)
{
  if (r == nullptr)
    r = new Record;
  try
  {
    while (symbol == literal)
    {
      std::string key = string;
      nextSymbol();
      //if(r->find(key) != r->end())
      //  throw std::logic_error(std::string("dublicate attribute '") + key + "'");
      expectSymbol(equals);
      if (symbol == literal)
      {
        Value*& key_value = (*r)[key];
        if (key_value != nullptr)
          delete key_value;
        key_value = new Literal(string);

        nextSymbol();
      }
      else if (symbol == lBrace)
      {
        nextSymbol();

        Value*& key_value = (*r)[key];
        Record* key_record = dynamic_cast<Record*>(key_value);
        if (key_record == nullptr && key_value != nullptr)
          delete key_value;
        parseRecord(key_record);
        key_value = key_record;

        expectSymbol(rBrace);
      }
      else if (symbol == lBracket)
      {
        Value*& key_value = (*r)[key];
        if (key_value != nullptr)
          delete key_value;
        key_value = parseArray();
      }
      else
        unexpectedSymbol();
      expectSymbol(semicolon);
    }
  }
  catch (const std::logic_error& e)
  {
    delete r;
    throw e;
  }
}

SimpleMap::Array* SimpleMap::parseArray()
{
  nextSymbol();
  Array* a = new Array();
  try
  {
    while (symbol == literal || symbol == lBrace)
    {
      if (symbol == literal)
      {
        a->push_back(new Literal(string));
        nextSymbol();
      }
      else if (symbol == lBrace)
      {
        nextSymbol();

        Array::iterator it = a->emplace(a->end());
        Record* index_record = dynamic_cast<Record*>(*it);
        parseRecord(index_record);
        *it = index_record;

        expectSymbol(rBrace);
      }
      if (symbol != rBracket)
        expectSymbol(comma);
    }
    expectSymbol(rBracket);
  }
  catch (const std::logic_error& e)
  {
    delete a;
    throw e;
  }
  return a;
}

void SimpleMap::parse(In& stream, const std::string& name)
{
  try
  {
    this->stream = &stream;
    c = 0;
    row = 1;
    column = 0;

    nextChar();
    nextSymbol();

    Record* root_record = dynamic_cast<Record*>(root);
    if (root_record == nullptr && root != nullptr)
      delete root;
    parseRecord(root_record);
    root = root_record;
  }
  catch (const std::logic_error& e)
  {
    root = nullptr;
    OUTPUT_ERROR(name << "(" << row << ", " << column << "): " << e.what());
  }
}

bool SimpleMap::Literal::operator==(const SimpleMap::Value& that) const
{
  const Literal* that_literal = dynamic_cast<const Literal*>(&that);
  if (!that_literal)
    return false;

  return this->literal == that_literal->literal;
}

bool SimpleMap::Record::operator==(const SimpleMap::Value& that) const
{
  const Record* that_record = dynamic_cast<const Record*>(&that);
  if (!that_record)
    return false;
  if (this->size() != that_record->size())
    return false;

  for (SimpleMap::Record::const_iterator first = this->begin(), second = that_record->begin(); first != this->end() && second != that_record->end(); ++first, ++second)
  {
    if (first->first != second->first || *(first->second) != *(second->second))
      return false;
  }

  return true;
}

bool SimpleMap::Array::operator==(const SimpleMap::Value& that) const
{
  const Array* that_array = dynamic_cast<const Array*>(&that);
  if (!that_array)
    return false;
  if (this->size() != that_array->size())
    return false;

  for (SimpleMap::Array::const_iterator first = this->begin(), second = that_array->begin(); first != this->end() && second != that_array->end(); ++first, ++second)
  {
    if (**first != **second)
      return false;
  }

  return true;
}

SimpleMap::Record& SimpleMap::Record::operator-=(const SimpleMap::Record& that)
{
  for (SimpleMap::Record::iterator it = this->begin(); it != this->end();)
  {
    if (that.find(it->first) == that.end())
    {
      ++it;
      continue;
    }

    SimpleMap::Value* this_element = it->second;
    const SimpleMap::Value* that_element = that.at(it->first);
    SimpleMap::Record* this_record = dynamic_cast<SimpleMap::Record*>(this_element);
    const SimpleMap::Record* that_record = dynamic_cast<const SimpleMap::Record*>(that_element);
    if (this_record != nullptr && that_record != nullptr)
    {
      *this_record -= *that_record;
      if (this_record->size() == 0)
      {
        delete this_record;
        it = this->erase(it);
      }
      else
        ++it;
    }
    else
    {
      if (*this_element == *that_element)
      {
        delete it->second;
        it = this->erase(it);
      }
      else
        ++it;
    }
  }
  return *this;
}

SimpleMap& SimpleMap::operator-=(const SimpleMap& that)
{
  SimpleMap::Record* this_record = dynamic_cast<SimpleMap::Record*>(this->root);
  const SimpleMap::Record* that_record = dynamic_cast<const SimpleMap::Record*>(that.root);
  ASSERT(this_record != nullptr);
  ASSERT(that_record != nullptr);

  *this_record -= *that_record;

  return *this;
}


void SimpleMap::serialize(In* in, Out* out)
{
  ASSERT(in == nullptr && out != nullptr);
  ASSERT(root);

  serialize(in, out, root);
}

void SimpleMap::serialize(In* in, Out* out, const SimpleMap::Value* value)
{
  ASSERT(value);

  if (const SimpleMap::Record* record = dynamic_cast<const SimpleMap::Record*>(value))
  {
    for (const auto& element : *record)
    {
      out->select(element.first.c_str(), dynamic_cast<const SimpleMap::Array*>(element.second) ? -1 : -2, nullptr);
      serialize(in, out, element.second);
      out->deselect();
    }
  }
  else if (const SimpleMap::Array* array = dynamic_cast<const SimpleMap::Array*>(value))
  {
    for (size_t i = 0; i < array->size(); i++)
    {
      out->select(0, static_cast<int>(i), nullptr);
      serialize(in, out, (*array)[i]);
      out->deselect();
    }
  }
  else if (const SimpleMap::Literal* literal = dynamic_cast<const SimpleMap::Literal*>(value))
  {
    std::string str;
    *literal >> str;
    *out << str;
  }
}

SimpleMap::SimpleMap(In& stream, const std::string& name) : stream(&stream), c(0), row(1), column(0), root(0)
{
  parse(stream, name);
}

SimpleMap::~SimpleMap()
{
  if (root)
    delete root;
}
