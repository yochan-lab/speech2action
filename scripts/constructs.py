#!/usr/bin/python

import re

reg_space = r' ?'
delim = '__'


class BaseSyntax(object):
    # default argument is an empty, mutable list. Generally bad practice, but it works a lot
    # better than global variables, while still allowing simple calls.
    def get_regex(self, names=[]):
        raise NotImplementedError

    def get_jsgf(self, use_names=True, weight=False):
        raise NotImplementedError

    def clean(self, *args):
        return map(self.clean_item, args)

    @staticmethod
    def clean_item(item):
        if isinstance(item, str):
            return StringSyntax(item)
        else:
            return item

    @staticmethod
    def strip_outer_parens(string):
        if string.startswith('(') and string.endswith(')'):
            return BaseSyntax.strip_outer_parens(string[1:-1])
        else:
            return string

    def get_names(self):
        return []

    def match(self, search_string):
        res = re.match(re.compile("^" + self.get_regex(names=[]) + "$"), str(search_string).lower())
        if not res:
            return
        gd = res.groupdict()
        out = {}
        for key, val in gd.iteritems():
            key = key.split(delim)[0]
            if out.has_key(key) and val:
                print key, val, out, gd
                raise NotImplementedError("Multiple identical parameters")
            elif val:
                out[key] = val
        return out

    def __or__(self, other):
        return OrSyntax(self, other)

    def __and__(self, other):
        return AndSyntax(self, other)

    def __add__(self, other):
        return AndSyntax(self, other)

    def __invert__(self):
        return OptionalSyntax(self)

    def __pos__(self):
        return OneOrMoreSyntax(self)

    def __neg__(self):
        return NoneOrMoreSyntax(self)

    def __mod__(self, other):
        return NamedSyntax(self, other)

    def __mul__(self, other):
        return WeightedSyntax(self, other)


class StringSyntax(BaseSyntax):
    def __init__(self, text, raw=False):
        self.text = str(text).lower()
        if raw:
            self.escaped = text
        else:
            self.escaped = re.escape(text)

    def get_regex(self, names=[]):
        return self.escaped

    def get_jsgf(self, use_names=True, weight=False):
        return self.text


S = StringSyntax


class OrSyntax(BaseSyntax):
    def __init__(self, *args):
        self.weighted = False
        self.children = self.clean(*args)

    def clean(self, *args):
        good = True
        newargs = []
        for arg in args:
            if isinstance(arg, OrSyntax):
                newargs.extend(self.clean(*arg.children))
                good = False
                continue
            elif isinstance(arg, WeightedSyntax):
                self.weighted = True
            newargs.append(self.clean_item(arg))
        if good:
            return newargs
        else:
            return self.clean(*newargs)

    def get_names(self):
        names = []
        for child in self.children:
            names.extend(child.get_names())
        return names

    def get_jsgf(self, use_names=True, weight=None):
        if self.weighted and weight is not False:
            weightedchildren = []
            for child in self.children:
                if isinstance(child, WeightedSyntax):
                    weightedchildren.append(child)
                else:
                    weightedchildren.append(WeightedSyntax(child, 1))
            jsgfs = map(lambda weightedchild: weightedchild.get_jsgf(use_names, weight=True), weightedchildren)
        else:
            jsgfs = map(lambda unweightedchild: unweightedchild.get_jsgf(use_names), self.children)
        return '(' + ' | '.join(jsgfs) + ')'

    def get_regex(self, names=[]):
        return '(' + '|'.join(map(lambda child: child.get_regex(names), self.children)) + ')'


class AndSyntax(BaseSyntax):
    def __init__(self, *args):
        self.children = self.clean(*args)

    def clean(self, *args):
        newargs = []
        for arg in args:
            if isinstance(arg, AndSyntax):
                newargs.extend(self.clean(*arg.children))
            else:
                newargs.append(self.clean_item(arg))
        return newargs

    def get_names(self):
        names = []
        for child in self.children:
            names.extend(child.get_names())
        return names

    def get_jsgf(self, use_names=True, weight=False):
        return ' '.join(map(lambda child: child.get_jsgf(use_names), self.children))

    def get_regex(self, names=[]):
        return reg_space.join(map(lambda child: child.get_regex(names), self.children))


class OptionalSyntax(BaseSyntax):
    def __init__(self, base):
        self.base = self.clean_item(base)

    def get_names(self):
        return self.base.get_names()

    def get_jsgf(self, use_names=True, weight=False):
        return '[' + self.strip_outer_parens(self.base.get_jsgf(use_names)) + ']'

    def get_regex(self, names=[]):
        return '(' + self.strip_outer_parens(self.base.get_regex(names)) + ')?'


class WeightedSyntax(BaseSyntax):
    def __init__(self, base, weight):
        self.base = self.clean_item(base)
        self.weight = weight

    def get_names(self):
        return self.base.get_names()

    def get_jsgf(self, use_names=True, weight=False):
        if weight:
            return '( /' + str(self.weight) + '/ ' + self.strip_outer_parens(self.base.get_jsgf(use_names)) + ')'
        else:
            return self.base.get_jsgf(use_names)

    def get_regex(self, names=[]):
        return self.base.get_regex(names)


class NoneOrMoreSyntax(BaseSyntax):
    def __init__(self, base):
        self.base = self.clean_item(base)

    def get_names(self):
        return self.base.get_names()

    def get_jsgf(self, use_names=True, weight=False):
        return '(' + self.strip_outer_parens(self.base.get_jsgf(use_names)) + ')*'

    def get_regex(self, names=[]):
        return '(' + self.base.get_regex(names) + ' ?)*'


class OneOrMoreSyntax(BaseSyntax):
    def __init__(self, base):
        self.base = self.clean_item(base)

    def get_names(self):
        return self.base.get_names()

    def get_jsgf(self, use_names=True, weight=False):
        return '(' + self.strip_outer_parens(self.base.get_jsgf(use_names)) + ')+'

    def get_regex(self, names=[]):
        return '(' + self.base.get_regex(names) + ' ?)+'


class NamedSyntax(BaseSyntax):
    def __init__(self, base, name):
        self.base = self.clean_item(base)
        self.name = name
        self.hacky_name = name

    def get_names(self):
        names = self.base.get_names()
        names.append(self.name)
        return names

    def get_jsgf(self, use_names=True, weight=False):
        if use_names:
            return '(%s) {%s}' % (self.strip_outer_parens(self.base.get_jsgf(use_names)), self.name)
        else:
            return self.base.get_jsgf(False)

    def get_regex(self, names=[]):
        if self.hacky_name in names:
            i = 0
            while self.hacky_name in names:
                i += 1
                self.hacky_name = self.name + delim + str(i)
        names.append(self.hacky_name)
        return '(?P<%s>%s)' % (self.hacky_name, self.base.get_regex(names))

class ArbitraryWordSyntax(BaseSyntax):
    def __init__(self, accept_numbers=True):
        self.accept_numbers = accept_numbers

    def get_regex(self, names=[]):
        if self.accept_numbers:
            return '\w'
        else:
            return '[A-Za-z]+'

    def get_jsgf(self, use_names=True, weight=False):
            raise RuntimeError("Finite state grammars do not support arbitrary inputs")

if __name__ == '__main__':
    nicegeneric = S('please') | 'will you'
    nicepre = nicegeneric | S('go ahead and')
    nicepost = nicegeneric | (~S('right') + 'now')
    cmd = -nicepre + (
        ((S('move') | 'go' | 'drive') % 'go' +
         ((S('right') | 'left') % 'dir' | S('to') + ~S('the') + (S('office') | 'cubicle' | 'kitchen') % 'place')) |
        (S('stop') | 'halt' | 'exit') % 'halt' |
        (S('spin') % 'spin' | S('turn') % 'go' + (S('around') | 'left' | 'right') % 'dir') |
        (S('say') | 'tell me' | 'speak') % 'say' + 'your' + (S('name') | 'identification') % 'info'
    ) + (-nicepost)
    print cmd.get_regex()
    print cmd.get_jsgf(use_names=False)
    print cmd.get_names()
    while True:
        s = raw_input('Match string: ')
        print s
        print cmd.match(s) or 'No match\n'
