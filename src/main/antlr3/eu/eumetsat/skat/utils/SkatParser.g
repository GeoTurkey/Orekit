parser grammar SkatParser;

options {
    output       = AST;
    ASTLabelType = CommonTree;
    tokenVocab   = SkatLexer;
}

tokens {
  STRUCT_VALUE;
  ARRAY_VALUE;
  BOOLEAN_VALUE;
  INT_VALUE;
  DOUBLE_VALUE;
  DATE_VALUE;
  STRING_VALUE;
}

@header {
    package eu.eumetsat.skat.utils;
}

data
    :  (a+=assignment (SEMICOLON)*)+ EOF
    -> ^(STRUCT_VALUE<SkatParseTree>[null, SkatParseTree.SkatType.STRUCTURE, $a]);

assignment
    : IDENTIFIER ASSIGN p=value[$IDENTIFIER.text]
    -> $p;

value[String key]
    : structValue[key]
    | arrayValue[key]
    | intValue[key]
    | doubleValue[key]
    | booleanValue[key]
    | stringValue[key];

structValue[String key]
    : STRUCT_OPEN (fields+=assignment (SEMICOLON)*)+ STRUCT_CLOSE
    -> ^(STRUCT_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.STRUCTURE, $fields]);

arrayValue[String key]
    : ARRAY_OPEN  elements+=value[null] (ARRAY_SEPARATOR elements+=value[null])* ARRAY_CLOSE
    -> ^(ARRAY_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.ARRAY, $elements]);

booleanValue[String key]
    : BOOLEAN
    -> ^(BOOLEAN_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.BOOLEAN]);

intValue[String key]
    : INT
    -> ^(INT_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.INT]);

doubleValue[String key]
    : DOUBLE
    -> ^(DOUBLE_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.DOUBLE]);

dateValue[String key]
    : DATE
    -> ^(DATE_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.DATE]);

stringValue[String key]
    : STRING
    -> ^(STRING_VALUE<SkatParseTree>[key, SkatParseTree.SkatType.STRING]);
