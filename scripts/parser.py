#!/usr/bin/python

from collections import namedtuple

__author__ = 'jmerdich'

import inspect
import copy


FullArgSpec = namedtuple('FullArgSpec', "name args defaults varargs kwargs")


class FnHintParser(object):
    all_terms = {}
    args = {}
    names = {}
    keywords = {}
    syntax = None

    # Returns name, args, default dict, *vararg, **kwarg
    @staticmethod
    def get_func_metadata(func):
        if not func:
            return
        spec = inspect.getargspec(func)
        if spec.defaults:
            defaults = dict(zip(reversed(spec.args), reversed(spec.defaults))).keys()
        else:
            defaults = None
        return FullArgSpec(func.func_name, spec.args, defaults, spec.varargs, spec.keywords)

    def checkfunc(self, kwarr, func):
        spec = self.get_func_metadata(func)
        # first, check we have everything the function requires
        if spec.args:
            if spec.defaults:
                args = copy.deepcopy(spec.args)
                for arg in spec.defaults:
                    args.remove(arg)
                required = args
            else:
                required = spec.args

            for arg in required:
                if arg not in kwarr:
                    return False
        return True

    def check_exact_func(self, kwarr, func):
        if not self.checkfunc(kwarr, func):
            return False
        spec = self.get_func_metadata(func)

        # else, check if we don't need to care about kw fitting in args
        if spec.kwargs:
            return True
        else:
            # if we do care, make sure they all fit (or are a function name)
            for kw in kwarr:
                if kw != spec.name and kw not in spec.args:
                    return False
        return True

    def valid_methods_w_weights(self, arr, exact=False):
        if isinstance(arr, dict):
            arr = arr.keys()
        selected_terms = []
        # remove anything that flat-out isn't there

        for kw in arr:
            if kw not in self.all_terms:
                arr.remove(kw)

        for kw in arr:
            for func in self.all_terms[kw]:
                if exact:
                    append = self.check_exact_func(arr, func[1])
                else:
                    append = self.checkfunc(arr, func[1])
                if append:
                    selected_terms.append(func)
        return selected_terms

    def valid_methods(self, arr, exact=False):
        method_w_weight = self.valid_methods_w_weights(arr, exact)
        output = []
        for method_tup in method_w_weight:
            if method_tup[1] not in output:
                output.append(method_tup[1])
        return output

    def pick_method(self, arr):
        exact = True
        method_w_weight = self.valid_methods_w_weights(arr, exact=True)
        if not method_w_weight:
            method_w_weight = self.valid_methods_w_weights(arr, exact=False)
            exact = False
        if not method_w_weight:
            return
        summed_weights = {}
        for method_tup in method_w_weight:
            try:
                summed_weights[method_tup[1]] = summed_weights[method_tup[1]] + method_tup[0]
            except KeyError:
                summed_weights[method_tup[1]] = method_tup[0]
        out_method, out_weight = summed_weights.popitem()
        for key in summed_weights:
            if out_weight < summed_weights[key]:
                out_weight = summed_weights[key]
                out_method = key
        print "func: " + out_method.func_name + "() [" + ("Exact" if exact else "Fuzzy") + ", %d]" % out_weight
        return out_method

    def pick_and_run(self, args):
        method = self.pick_method(args)
        if not method:
            print "No method found for" + str(args)
            return
        spec = self.get_func_metadata(method)
        params = {}
        varargs = []
        for arg in args:
            if arg in spec.args:
                params[arg] = args[arg]
            elif arg == spec.name:
                continue
            elif spec.kwargs:
                params[arg] = args[arg]
        return method(*varargs, **params)

    def register_syntax(self, syntax):
        if self.syntax:
            self.syntax = self.syntax | syntax
        else:
            self.syntax = syntax

    def parse(self, command):
        return self.pick_method(self.syntax.match(command))

    def parse_and_run(self, command):
        keywords = self.syntax.match(command)
        print  "      " + str(keywords)
        if keywords:
            return self.pick_and_run(keywords)

    # Set weight by argument using yourargname_weight
    def register_fn(self, keywords=None,
                    name_weight=2, kw_weight=4, arg_weight=5, arg_w_default_weight=1, **kwargs):
        if isinstance(keywords, str):
            keywords = [keywords]

        def add_to_dict(d, k, v):
            try:
                if d[k]:
                    d[k].append(v)
            except KeyError:
                d[k] = [v]

        def wrapper(func):
            spec = self.get_func_metadata(func)

            add_to_dict(self.all_terms, spec.name, (name_weight, func))
            add_to_dict(self.names, spec.name, (name_weight, func))

            if keywords:
                for kw in keywords:
                    add_to_dict(self.all_terms, kw, (kw_weight, func))
                    add_to_dict(self.keywords, kw, (kw_weight, func))

            for arg in spec.args:
                if (arg + '_weight') in kwargs:
                    weight = kwargs[arg+'_weight']
                elif spec.defaults and arg in spec.defaults:
                    weight = arg_w_default_weight
                else:
                    weight = arg_weight
                add_to_dict(self.all_terms, arg, (weight, func))
                add_to_dict(self.args, arg, (weight, func))
            return func
        return wrapper

    def export_jsgf(self, identifier='cmd', namespace='cmd'):
        head = "#JSGF V1.0;\n" \
               "\n" \
               "grammar %s;\n" \
               "\n" \
               "public <%s> =" % (namespace, identifier)
        tail = ";"
        if self.syntax:
            return head + self.syntax.get_jsgf() + tail
