parser grammar SkatParser;

options {
    output       = AST;
    ASTLabelType = CommonTree;
    tokenVocab   = SkatLexer;
}

tokens {
  ASSIGNMENT;
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
    -> ^(STRUCT_VALUE $a);

assignment
    : IDENTIFIER ASSIGN v=value
    -> ^(ASSIGNMENT $v);

value
    : structValue
    | arrayValue
    | INT
    | DOUBLE
    | DATE
    | BOOLEAN
    | STRING;

structValue
    : STRUCT_OPEN (fields+=assignment (SEMICOLON)*)+ STRUCT_CLOSE
    -> ^(STRUCT_VALUE  $fields);

arrayValue
    : ARRAY_OPEN  elements+=value (ARRAY_SEPARATOR elements+=value)* ARRAY_CLOSE
    -> ^(ARRAY_VALUE  $elements);
