/*
 * SyntaxHighlighter.cpp
 *
 *  Created on: 25.09.2010
 *      Author: Emil Huseynli (Originally: Martin Holmes, Meagan Timney, University of Victoria, HCMC)
 *
 */

#include "SyntaxHighlighter.h"

SyntaxHighlighter::SyntaxHighlighter(QTextDocument* parent) : QSyntaxHighlighter(parent)
{
  HighlightingRule rule;

  const QString nameStartCharList = ":A-Z_a-z";
  const QString nameCharList = nameStartCharList + "\\-\\.0-9";
  const QString nameStart = "[" + nameStartCharList + "]";
  const QString nameChar = "[" + nameCharList + "]";
  const QString xmlName = nameStart + "(" + nameChar + ")*";

  //Processing instructions.
  xmlProcInstStartExpression = QRegularExpression("<\\?");
  xmlProcInstEndExpression = QRegularExpression("\\?>");
  xmlProcInstFormat.setForeground(Qt::red);
  xmlProcInstFormat.setFontItalic(false);

  //Multiline comments.
  xmlCommentStartExpression = QRegularExpression("<!\\-\\-");
  xmlCommentEndExpression = QRegularExpression("\\-\\->");
  xmlCommentFormat.setForeground(Qt::darkGreen);
  xmlCommentFormat.setFontItalic(false);

  //Opening tags
  xmlOpenTagStartExpression = QRegularExpression("<" + xmlName);
  xmlOpenTagEndExpression = QRegularExpression(">");

  //Closing tags: first shot, handling them as start-end pairs.
  xmlCloseTagStartExpression = QRegularExpression("</" + xmlName);
  xmlCloseTagEndExpression = QRegularExpression(">");

  xmlTagFormat.setForeground(Qt::darkBlue);
  xmlTagFormat.setFontItalic(false);

  //Attributes
  xmlAttributeStartExpression = QRegularExpression("\\s*" + xmlName + "\\s*=\\s*\\\"");
  xmlAttributeEndExpression = QRegularExpression("(?<!\\\\)(?:(\\\\\\\\)*)(\")");
  xmlAttributeFormat.setForeground(Qt::darkMagenta);
  xmlAttributeFormat.setFontItalic(false);

  //The end expression varies depending on what's matched with the start expression (single or double quote).
  //This regexp is actually reset repeatedly during the highlighting process.
  xmlAttValFormat.setForeground(QColor(139, 69, 19));
  xmlAttValFormat.setFontItalic(false);

  //The DOCTYPE declaration.
  xmlDoctypeStartExpression = QRegularExpression("<!DOCTYPE");
  xmlDoctypeEndExpression = QRegularExpression(">");
  xmlDoctypeFormat.setForeground(Qt::darkCyan);
  xmlDoctypeFormat.setFontItalic(false);

  //Next, XML entities. This is the first item we can assume not to be multiline,
  //So this can be added to the rules. We add this to the second set of rules so
  //that it can highlight inside attribute values.
  xmlEntityFormat.setForeground(Qt::darkGray);
  xmlEntityFormat.setFontItalic(true);
  rule.pattern = QRegularExpression("&" + xmlName + ";");
  rule.format = xmlEntityFormat;
  hlRules.append(rule);
}

void SyntaxHighlighter::highlightBlock(const QString& text)
{
  //Do the main block highlighting.
  highlightSubBlock(text, 0, previousBlockState());

  //Run the set of inline rules.
  foreach (const HighlightingRule& rule, hlRules)
  {
    QRegularExpression expression(rule.pattern);

    for (const auto& match : expression.globalMatch(text))
    {
      setFormat(match.capturedStart(), match.capturedLength(), rule.format);
    }
  }
}

void SyntaxHighlighter::highlightSubBlock(const QString& text, const int startIndex, const int currState)
{
  if (startIndex >= text.length())
    return;
  int lowest = -1;
  int newState = -1;

  int effectiveState = currState;
  if (currState < 0)
    effectiveState = inNothing;
  switch (effectiveState)
  {
  case (inNothing):
  {
    //If we're not in anything, then what could be coming is either a comment, processing instruction, a doctype decl, or a tag (open or close).
    const auto comment = xmlCommentStartExpression.match(text, startIndex);
    const auto procInst = xmlProcInstStartExpression.match(text, startIndex);
    const auto doctype = xmlDoctypeStartExpression.match(text, startIndex);
    const auto openTag = xmlOpenTagStartExpression.match(text, startIndex);
    const auto closeTag = xmlCloseTagStartExpression.match(text, startIndex);
    if (comment.hasMatch())
    {
      lowest = comment.capturedStart();
      newState = inComment;
    }
    if (procInst.hasMatch() && ((lowest == -1) || (procInst.capturedStart() < lowest)))
    {
      lowest = procInst.capturedStart();
      newState = inProcInst;
    }
    if (doctype.hasMatch() && ((lowest == -1) || (doctype.capturedStart() < lowest)))
    {
      lowest = doctype.capturedStart();
      newState = inDoctypeDecl;
    }
    if (openTag.hasMatch() && ((lowest == -1) || (openTag.capturedStart() < lowest)))
    {
      lowest = openTag.capturedStart();
      newState = inOpenTag;
    }
    if (closeTag.hasMatch() && ((lowest == -1) || (closeTag.capturedStart() < lowest)))
    {
      newState = inCloseTag;
    }
    switch (newState)
    {
    case -1:
    {
      //Nothing starts in this block.
      setCurrentBlockState(inNothing);
      break;
    }
    case inComment:
    {
      //We're into a comment.
      setFormat(comment.capturedStart(), comment.capturedLength(), xmlCommentFormat);
      setCurrentBlockState(inComment);
      highlightSubBlock(text, comment.capturedEnd(), inComment);
      break;
    }
    case inProcInst:
    {
      //We're into a processing instruction.
      //Format the matched text
      setFormat(procInst.capturedStart(), procInst.capturedLength(), xmlProcInstFormat);
      //Call this function again with a new offset and state.
      setCurrentBlockState(inProcInst);
      highlightSubBlock(text, procInst.capturedEnd(), inProcInst);
      break;
    }
    case inDoctypeDecl:
    {
      //We're into a document type declaration.
      //Format the matched text
      setFormat(doctype.capturedStart(), doctype.capturedLength(), xmlDoctypeFormat);
      //Call this function again with a new offset and state.
      setCurrentBlockState(inDoctypeDecl);
      highlightSubBlock(text, doctype.capturedEnd(), inDoctypeDecl);
      break;
    }
    case inOpenTag:
    {
      //We're into an opening tag.
      //Format the matched text
      setFormat(openTag.capturedStart(), openTag.capturedLength(), xmlTagFormat);
      //Call this function again with a new offset and state.
      setCurrentBlockState(inOpenTag);
      highlightSubBlock(text, openTag.capturedEnd(), inOpenTag);
      break;
    }
    case inCloseTag:
    {
      //We're into a closing tag.
      //Format the matched text
      setFormat(closeTag.capturedStart(), closeTag.capturedLength(), xmlTagFormat);
      setCurrentBlockState(inCloseTag);
      //Call this function again with a new offset and state.
      highlightSubBlock(text, closeTag.capturedEnd(), inCloseTag);
      break;
    }
    }
    break;
  }
  case inProcInst:
  {
    //Look for the end of the processing instruction.
    const auto offset = xmlProcInstEndExpression.match(text, startIndex);
    if (offset.hasMatch())
    {
      setFormat(startIndex, offset.capturedEnd() - startIndex, xmlProcInstFormat);
      setCurrentBlockState(inNothing);
      highlightSubBlock(text, offset.capturedEnd(), inNothing);
    }
    else
    {
      //We leave this block still inside the processing instruction,
      //after formatting the rest of the line.
      setFormat(startIndex, text.length() - startIndex, xmlProcInstFormat);
      setCurrentBlockState(inProcInst);
    }
    break;
  }
  case inDoctypeDecl:
  {
    //Look for the end of the doctype declaration.
    const auto offset = xmlDoctypeEndExpression.match(text, startIndex);
    if (offset.hasMatch())
    {
      setFormat(startIndex, offset.capturedEnd() - startIndex, xmlDoctypeFormat);
      setCurrentBlockState(inNothing);
      highlightSubBlock(text, offset.capturedEnd(), inNothing);
    }
    else
    {
      //We leave this block still inside the doctype declaration,
      //after formatting the rest of the line.
      setFormat(startIndex, text.length() - startIndex, xmlDoctypeFormat);
      setCurrentBlockState(inDoctypeDecl);
    }
    break;
  }
  case inOpenTag:
  {
    //If we're in an open tag, we're looking either for the end of the open tag, or for
    //the beginning of an attribute name.
    const auto openTagEnd = xmlOpenTagEndExpression.match(text, startIndex);
    const auto attStart = xmlAttributeStartExpression.match(text, startIndex);
    if (attStart.hasMatch())
    {
      lowest = attStart.capturedStart();
      newState = inAttVal;
    }
    if (openTagEnd.hasMatch() && ((lowest == -1) || (openTagEnd.capturedStart() < lowest)))
    {
      newState = inNothing;
    }
    switch (newState)
    {
    case -1:
    {
      //we're still in a tag. No need to highlight anything.
      setCurrentBlockState(inOpenTag);
      break;
    }
    case inNothing:
    {
      //We've come to the end of the open tag.
      setFormat(openTagEnd.capturedStart(), openTagEnd.capturedLength(), xmlTagFormat);
      setCurrentBlockState(inNothing);
      highlightSubBlock(text, openTagEnd.capturedEnd(), inNothing);
      break;
    }
    case inAttVal:
    {
      //We've started an attribute. First format the attribute name and quote.
      setFormat(attStart.capturedStart(), attStart.capturedLength(), xmlAttributeFormat);
      setCurrentBlockState(inAttVal);
      highlightSubBlock(text, attStart.capturedEnd(), inAttVal);
      break;
    }
    }
    break;
  }
  case inAttVal:
  {
    //When we're in an attribute value, we're only looking for the closing quote.
    const auto attrEnd = xmlAttributeEndExpression.match(text, startIndex);
    if (attrEnd.hasMatch())
    {
      //Do some highlighting. First the attribute value.
      setFormat(startIndex, attrEnd.capturedStart(2) - startIndex, xmlAttValFormat);
      //Now the closing quote.
      setFormat(attrEnd.capturedStart(2), attrEnd.capturedLength(2), xmlAttributeFormat);
      setCurrentBlockState(inOpenTag);
      highlightSubBlock(text, attrEnd.capturedStart() + 1, inOpenTag);
    }
    else
    {
      //The attribute value runs over the end of the line.
      setFormat(startIndex, text.length() - startIndex, xmlAttValFormat);
      setCurrentBlockState(inAttVal);
    }
    break;
  }
  case inCloseTag:
  {
    const auto closeTagEnd = xmlCloseTagEndExpression.match(text, startIndex);
    if (closeTagEnd.hasMatch())
    {
      //We've found the end of the close tag.
      setFormat(closeTagEnd.capturedStart(), closeTagEnd.capturedLength(), xmlTagFormat);
      setCurrentBlockState(inNothing);
      highlightSubBlock(text, closeTagEnd.capturedEnd(), inNothing);
    }
    else
    {
      //There must be a linebreak inside the close tag.
      setCurrentBlockState(inCloseTag);
    }
    break;
  }
  case inComment:
  {
    //Once we're in a comment, we just have to search for the end of it. Nothing else takes precedence.
    //Look for the end of the comment.
    const auto offset = xmlCommentEndExpression.match(text, startIndex);
    if (offset.hasMatch())
    {
      setFormat(startIndex, offset.capturedEnd() - startIndex, xmlCommentFormat);
      setCurrentBlockState(inNothing);
      highlightSubBlock(text, offset.capturedEnd(), inNothing);
    }
    else
    {
      //We leave this block still inside the comment,
      //after formatting the rest of the line.
      setFormat(startIndex, text.length() - startIndex, xmlCommentFormat);
      setCurrentBlockState(inComment);
    }
    break;
  }
  }
}

void SyntaxHighlighter::setCharFormat(const int whichFormat, const QTextCharFormat& newFormat)
{
  QTextCharFormat* targFormat;
  switch (whichFormat)
  {
  case xmlDefault:
  {
    targFormat = &xmlDefaultFormat;
    break;
  }
  case xmlProcInst:
  {
    targFormat = &xmlProcInstFormat;
    break;
  }
  case xmlDoctype:
  {
    targFormat = &xmlDoctypeFormat;
    break;
  }
  case xmlComment:
  {
    targFormat = &xmlCommentFormat;
    break;
  }
  case xmlTag:
  {
    targFormat = &xmlTagFormat;
    break;
  }
  case xmlEntity:
  {
    targFormat = &xmlEntityFormat;
    break;
  }
  case xmlAttribute:
  {
    targFormat = &xmlAttributeFormat;
    break;
  }
  case xmlAttVal:
  {
    targFormat = &xmlAttValFormat;
    break;
  }
  default:
  {
    return;
  }
  }

  targFormat->setForeground(newFormat.foreground());
  rehighlight();
}

bool SyntaxHighlighter::getCharFormat(const int whichFormat, QTextCharFormat& targFormat)
{
  QTextCharFormat* srcFormat;
  switch (whichFormat)
  {
  case xmlDefault:
  {
    srcFormat = &xmlDefaultFormat;
    break;
  }
  case xmlProcInst:
  {
    srcFormat = &xmlProcInstFormat;
    break;
  }
  case xmlDoctype:
  {
    srcFormat = &xmlDoctypeFormat;
    break;
  }
  case xmlComment:
  {
    srcFormat = &xmlCommentFormat;
    break;
  }
  case xmlTag:
  {
    srcFormat = &xmlTagFormat;
    break;
  }
  case xmlEntity:
  {
    srcFormat = &xmlEntityFormat;
    break;
  }
  case xmlAttribute:
  {
    srcFormat = &xmlAttributeFormat;
    break;
  }
  case xmlAttVal:
  {
    srcFormat = &xmlAttValFormat;
    break;
  }
  default:
  {
    return false;
  }
  }

  targFormat.setForeground(srcFormat->foreground());

  return true;
}
