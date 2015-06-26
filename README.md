# speech2action

Interprets a strict-language string and translates that to the best registered action based on hints and introspection. Supports compiling down to regex and JSGF (for PocketSphinx).

## How to Use

All code goes into the `actions.py` file. 

1. Instanciate a parser with FnHintParser(). Multiple grammars are supported by using multiple parsers, otherwise they are just chained together (OR).
2. Register actions by decorating a function with `@parser_name.register_fn()`.
3. Register command syntaxes with `parser_name.register_syntax(syntax)`
4. Find matches with `parser_name.parse_and_run(in_string)` or other parser methods.

## Syntax
Syntax is created using Python objects, but there are shortcuts to ease writing it.

- **String**: (only required if using shortcuts) StringSyntax(str) or `S(str)`
- **OR**: OrSyntax(a,b...) or `a|b`
- **AND**: AndSyntax(a,b...) or `a+b` or `a&b`
- **Optional**: OptionalSyntax(syntax) or `~syntax`
- **0+**: ZeroOrMoreSyntax(syntax) or `-syntax`
- **1+**: OneOrMoreSyntax(syntax) or `+syntax`
- **Weighted**: WeightedSyntax(syntax, weight)
- **Hinted**: NamedSyntax(syntax, name) or `syntax % name`
- **Arbitrary**: (not allowed with JSGF) ArbitraryWordSyntax(accept_numbers=True)

## ROS
The default actions.py listens for input on 'recognizer/speech' by default (configurable on ~speech_in)
