parser grammar SkatParser;

options {
    output       = AST;
    ASTLabelType = CommonTree;
    tokenVocab   = SkatLexer;
}

tokens {
  ASSIGNMENT;
  STRUCT;
  ARRAY;
}

@header {
    package eu.eumetsat.skat.utils;
}

data
    :  a+=assignment (SEMICOLON a+=assignment)* SEMICOLON* EOF
    -> ^(STRUCT $a+);

assignment
    : IDENTIFIER ASSIGN v=value
    -> ^(ASSIGNMENT IDENTIFIER $v);

value
    : structValue
    | arrayValue
    | IDENTIFIER
    | INT
    | DOUBLE
    | DATE
    | BOOLEAN
    | STRING;

structValue
    : STRUCT_OPEN fields+=assignment (SEMICOLON fields+=assignment)* SEMICOLON* STRUCT_CLOSE
    -> ^(STRUCT  $fields+);

arrayValue
    : ARRAY_OPEN  elements+=value (ARRAY_SEPARATOR elements+=value)* ARRAY_SEPARATOR* ARRAY_CLOSE
    -> ^(ARRAY  $elements+);
