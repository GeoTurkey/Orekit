// Process this file with antlr to produce a lexer

lexer grammar SkatLexer;

@header {
    package eu.eumetsat.skat.utils;
}

@members {

    private String unquote(String quoted) {
        final StringBuilder unquoted = new StringBuilder();
        for (int i = 1; i < quoted.length() - 1; ++i) {
            if (quoted.charAt(i) == '\\') {
                ++i;
            }
            unquoted.append(quoted.charAt(i));
        }
        return unquoted.toString();
    }

}


TRUE                          : 't' 'r' 'u' 'e';
FALSE                         : 'f' 'a' 'l' 's' 'e';

fragment LETTER_LIKE          : ('a'..'z') | ('A'..'Z') | '_';
fragment DIGIT                : '0'..'9';
fragment SHARP                : '#';
IDENTIFIER                    : LETTER_LIKE (LETTER_LIKE | DIGIT)*;

WHITESPACE_OR_NEWLINE         : (' ' | '\t' | '\r' | '\n') { skip(); };
COMMENT                       : SHARP (~'\n')* { skip(); };

STRUCT_OPEN                   : '{' ;
STRUCT_CLOSE                  : '}' ;
ARRAY_OPEN                    : '[' ;
ARRAY_CLOSE                   : ']' ;
ARRAY_SEPARATOR               : ',' ;
ASSIGN                        : '=' ;
SEMICOLON                     : ';' ;

fragment SIGN                 : ('+' | '-');
INT                           : SIGN? DIGIT+;

fragment DOTTED_MANTISSA      : (DIGIT+ '.' DIGIT*) | ('.' DIGIT+);
fragment EXPONENT             : ('e' | 'E') SIGN? DIGIT+;
DOUBLE                        : SIGN? ((DOTTED_MANTISSA (EXPONENT)?) | (DIGIT+ EXPONENT));

fragment DAY                  : DIGIT DIGIT DIGIT DIGIT '-' DIGIT DIGIT '-' DIGIT DIGIT;
fragment HOUR                 : DIGIT DIGIT ':' DIGIT DIGIT ':' DIGIT DIGIT ('.' DIGIT+)?;
DATE                          : DAY 'T' HOUR;

fragment QUOTED_STRING        : '"'  (('\\' .) | ~('\\' | '"'))* '"';
STRING                        : QUOTED_STRING { setText(unquote($text)); };
