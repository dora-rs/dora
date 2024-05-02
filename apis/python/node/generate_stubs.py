import argparse
import ast
import importlib
import inspect
import logging
import re
import subprocess
from functools import reduce
from typing import Any, Dict, List, Mapping, Optional, Set, Tuple, Union


def path_to_type(*elements: str) -> ast.AST:
    base: ast.AST = ast.Name(id=elements[0], ctx=ast.Load())
    for e in elements[1:]:
        base = ast.Attribute(value=base, attr=e, ctx=ast.Load())
    return base


OBJECT_MEMBERS = dict(inspect.getmembers(object))
BUILTINS: Dict[str, Union[None, Tuple[List[ast.AST], ast.AST]]] = {
    "__annotations__": None,
    "__bool__": ([], path_to_type("bool")),
    "__bytes__": ([], path_to_type("bytes")),
    "__class__": None,
    "__contains__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__del__": None,
    "__delattr__": ([path_to_type("str")], path_to_type("None")),
    "__delitem__": ([path_to_type("typing", "Any")], path_to_type("typing", "Any")),
    "__dict__": None,
    "__dir__": None,
    "__doc__": None,
    "__eq__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__format__": ([path_to_type("str")], path_to_type("str")),
    "__ge__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__getattribute__": ([path_to_type("str")], path_to_type("typing", "Any")),
    "__getitem__": ([path_to_type("typing", "Any")], path_to_type("typing", "Any")),
    "__gt__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__hash__": ([], path_to_type("int")),
    "__init__": ([], path_to_type("None")),
    "__init_subclass__": None,
    "__iter__": ([], path_to_type("typing", "Any")),
    "__le__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__len__": ([], path_to_type("int")),
    "__lt__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__module__": None,
    "__ne__": ([path_to_type("typing", "Any")], path_to_type("bool")),
    "__new__": None,
    "__next__": ([], path_to_type("typing", "Any")),
    "__int__": ([], path_to_type("None")),
    "__reduce__": None,
    "__reduce_ex__": None,
    "__repr__": ([], path_to_type("str")),
    "__setattr__": (
        [path_to_type("str"), path_to_type("typing", "Any")],
        path_to_type("None"),
    ),
    "__setitem__": (
        [path_to_type("typing", "Any"), path_to_type("typing", "Any")],
        path_to_type("typing", "Any"),
    ),
    "__sizeof__": None,
    "__str__": ([], path_to_type("str")),
    "__subclasshook__": None,
}


def module_stubs(module: Any) -> ast.Module:
    types_to_import = {"typing"}
    classes = []
    functions = []
    for member_name, member_value in inspect.getmembers(module):
        element_path = [module.__name__, member_name]
        if member_name.startswith("__"):
            pass
        elif member_name.startswith("DoraStatus"):
            pass
        elif inspect.isclass(member_value):
            classes.append(
                class_stubs(member_name, member_value, element_path, types_to_import)
            )
        elif inspect.isbuiltin(member_value):
            functions.append(
                function_stub(
                    member_name,
                    member_value,
                    element_path,
                    types_to_import,
                    in_class=False,
                )
            )
        else:
            logging.warning(f"Unsupported root construction {member_name}")
    return ast.Module(
        body=[ast.Import(names=[ast.alias(name=t)]) for t in sorted(types_to_import)]
        + classes
        + functions,
        type_ignores=[],
    )


def class_stubs(
    cls_name: str, cls_def: Any, element_path: List[str], types_to_import: Set[str]
) -> ast.ClassDef:
    attributes: List[ast.AST] = []
    methods: List[ast.AST] = []
    magic_methods: List[ast.AST] = []
    constants: List[ast.AST] = []
    for member_name, member_value in inspect.getmembers(cls_def):
        current_element_path = [*element_path, member_name]
        if member_name == "__init__":
            try:
                inspect.signature(cls_def)  # we check it actually exists
                methods = [
                    function_stub(
                        member_name,
                        cls_def,
                        current_element_path,
                        types_to_import,
                        in_class=True,
                    ),
                    *methods,
                ]
            except ValueError as e:
                if "no signature found" not in str(e):
                    raise ValueError(
                        f"Error while parsing signature of {cls_name}.__init_"
                    ) from e
        elif (
            member_value == OBJECT_MEMBERS.get(member_name)
            or BUILTINS.get(member_name, ()) is None
        ):
            pass
        elif inspect.isdatadescriptor(member_value):
            attributes.extend(
                data_descriptor_stub(
                    member_name, member_value, current_element_path, types_to_import
                )
            )
        elif inspect.isroutine(member_value):
            (magic_methods if member_name.startswith("__") else methods).append(
                function_stub(
                    member_name,
                    member_value,
                    current_element_path,
                    types_to_import,
                    in_class=True,
                )
            )
        elif member_name == "__match_args__":
            constants.append(
                ast.AnnAssign(
                    target=ast.Name(id=member_name, ctx=ast.Store()),
                    annotation=ast.Subscript(
                        value=path_to_type("tuple"),
                        slice=ast.Tuple(
                            elts=[path_to_type("str"), ast.Ellipsis()], ctx=ast.Load()
                        ),
                        ctx=ast.Load(),
                    ),
                    value=ast.Constant(member_value),
                    simple=1,
                )
            )
        elif member_value is not None:
            constants.append(
                ast.AnnAssign(
                    target=ast.Name(id=member_name, ctx=ast.Store()),
                    annotation=concatenated_path_to_type(
                        member_value.__class__.__name__, element_path, types_to_import
                    ),
                    value=ast.Ellipsis(),
                    simple=1,
                )
            )
        else:
            logging.warning(
                f"Unsupported member {member_name} of class {'.'.join(element_path)}"
            )

    doc = inspect.getdoc(cls_def)
    doc_comment = build_doc_comment(doc) if doc else None
    return ast.ClassDef(
        cls_name,
        bases=[],
        keywords=[],
        body=(
            ([doc_comment] if doc_comment else [])
            + attributes
            + methods
            + magic_methods
            + constants
        )
        or [ast.Ellipsis()],
        decorator_list=[path_to_type("typing", "final")],
    )


def data_descriptor_stub(
    data_desc_name: str,
    data_desc_def: Any,
    element_path: List[str],
    types_to_import: Set[str],
) -> Union[Tuple[ast.AnnAssign, ast.Expr], Tuple[ast.AnnAssign]]:
    annotation = None
    doc_comment = None

    doc = inspect.getdoc(data_desc_def)
    if doc is not None:
        annotation = returns_stub(data_desc_name, doc, element_path, types_to_import)
        m = re.findall(r"^ *:return: *(.*) *$", doc, re.MULTILINE)
        if len(m) == 1:
            doc_comment = m[0]
        elif len(m) > 1:
            raise ValueError(
                f"Multiple return annotations found with :return: in {'.'.join(element_path)} documentation"
            )

    assign = ast.AnnAssign(
        target=ast.Name(id=data_desc_name, ctx=ast.Store()),
        annotation=annotation or path_to_type("typing", "Any"),
        simple=1,
    )
    doc_comment = build_doc_comment(doc_comment) if doc_comment else None
    return (assign, doc_comment) if doc_comment else (assign,)


def function_stub(
    fn_name: str,
    fn_def: Any,
    element_path: List[str],
    types_to_import: Set[str],
    *,
    in_class: bool,
) -> ast.FunctionDef:
    body: List[ast.AST] = []
    doc = inspect.getdoc(fn_def)
    if doc is not None:
        doc_comment = build_doc_comment(doc)
        if doc_comment is not None:
            body.append(doc_comment)

    decorator_list = []
    if in_class and hasattr(fn_def, "__self__"):
        decorator_list.append(ast.Name("staticmethod"))

    return ast.FunctionDef(
        fn_name,
        arguments_stub(fn_name, fn_def, doc or "", element_path, types_to_import),
        body or [ast.Ellipsis()],
        decorator_list=decorator_list,
        returns=(
            returns_stub(fn_name, doc, element_path, types_to_import) if doc else None
        ),
        lineno=0,
    )


def arguments_stub(
    callable_name: str,
    callable_def: Any,
    doc: str,
    element_path: List[str],
    types_to_import: Set[str],
) -> ast.arguments:
    real_parameters: Mapping[str, inspect.Parameter] = inspect.signature(
        callable_def
    ).parameters
    if callable_name == "__init__":
        real_parameters = {
            "self": inspect.Parameter("self", inspect.Parameter.POSITIONAL_ONLY),
            **real_parameters,
        }

    parsed_param_types = {}
    optional_params = set()

    # Types for magic functions types
    builtin = BUILTINS.get(callable_name)
    if isinstance(builtin, tuple):
        param_names = list(real_parameters.keys())
        if param_names and param_names[0] == "self":
            del param_names[0]
        for name, t in zip(param_names, builtin[0]):
            parsed_param_types[name] = t

    # Types from comment
    for match in re.findall(
        r"^ *:type *([a-zA-Z0-9_]+): ([^\n]*) *$", doc, re.MULTILINE
    ):
        if match[0] not in real_parameters:
            raise ValueError(
                f"The parameter {match[0]} of {'.'.join(element_path)} "
                "is defined in the documentation but not in the function signature"
            )
        type = match[1]
        if type.endswith(", optional"):
            optional_params.add(match[0])
            type = type[:-10]
        parsed_param_types[match[0]] = convert_type_from_doc(
            type, element_path, types_to_import
        )

    # we parse the parameters
    posonlyargs = []
    args = []
    vararg = None
    kwonlyargs = []
    kw_defaults = []
    kwarg = None
    defaults = []
    for param in real_parameters.values():
        if param.name != "self" and param.name not in parsed_param_types:
            raise ValueError(
                f"The parameter {param.name} of {'.'.join(element_path)} "
                "has no type definition in the function documentation"
            )
        param_ast = ast.arg(
            arg=param.name, annotation=parsed_param_types.get(param.name)
        )

        default_ast = None
        if param.default != param.empty:
            default_ast = ast.Constant(param.default)
            if param.name not in optional_params:
                raise ValueError(
                    f"Parameter {param.name} of {'.'.join(element_path)} "
                    "is optional according to the type but not flagged as such in the doc"
                )
        elif param.name in optional_params:
            raise ValueError(
                f"Parameter {param.name} of {'.'.join(element_path)} "
                "is optional according to the documentation but has no default value"
            )

        if param.kind == param.POSITIONAL_ONLY:
            args.append(param_ast)
            # posonlyargs.append(param_ast)
            # defaults.append(default_ast)
        elif param.kind == param.POSITIONAL_OR_KEYWORD:
            args.append(param_ast)
            defaults.append(default_ast)
        elif param.kind == param.VAR_POSITIONAL:
            vararg = param_ast
        elif param.kind == param.KEYWORD_ONLY:
            kwonlyargs.append(param_ast)
            kw_defaults.append(default_ast)
        elif param.kind == param.VAR_KEYWORD:
            kwarg = param_ast

    return ast.arguments(
        posonlyargs=posonlyargs,
        args=args,
        vararg=vararg,
        kwonlyargs=kwonlyargs,
        kw_defaults=kw_defaults,
        defaults=defaults,
        kwarg=kwarg,
    )


def returns_stub(
    callable_name: str, doc: str, element_path: List[str], types_to_import: Set[str]
) -> Optional[ast.AST]:
    m = re.findall(r"^ *:rtype: *([^\n]*) *$", doc, re.MULTILINE)
    if len(m) == 0:
        builtin = BUILTINS.get(callable_name)
        if isinstance(builtin, tuple) and builtin[1] is not None:
            return builtin[1]
        raise ValueError(
            f"The return type of {'.'.join(element_path)} "
            "has no type definition using :rtype: in the function documentation"
        )
    if len(m) > 1:
        raise ValueError(
            f"Multiple return type annotations found with :rtype: for {'.'.join(element_path)}"
        )
    return convert_type_from_doc(m[0], element_path, types_to_import)


def convert_type_from_doc(
    type_str: str, element_path: List[str], types_to_import: Set[str]
) -> ast.AST:
    type_str = type_str.strip()
    return parse_type_to_ast(type_str, element_path, types_to_import)


def parse_type_to_ast(
    type_str: str, element_path: List[str], types_to_import: Set[str]
) -> ast.AST:
    # let's tokenize
    tokens = []
    current_token = ""
    for c in type_str:
        if "a" <= c <= "z" or "A" <= c <= "Z" or c == ".":
            current_token += c
        else:
            if current_token:
                tokens.append(current_token)
            current_token = ""
            if c != " ":
                tokens.append(c)
    if current_token:
        tokens.append(current_token)

    # let's first parse nested parenthesis
    stack: List[List[Any]] = [[]]
    for token in tokens:
        if token == "[":
            children: List[str] = []
            stack[-1].append(children)
            stack.append(children)
        elif token == "]":
            stack.pop()
        else:
            stack[-1].append(token)

    # then it's easy
    def parse_sequence(sequence: List[Any]) -> ast.AST:
        # we split based on "or"
        or_groups: List[List[str]] = [[]]
        print(sequence)
        # TODO: Fix sequence
        if "Ros" in sequence and "2" in sequence:
            sequence = ["".join(sequence)]
        elif "dora.Ros" in sequence and "2" in sequence:
            sequence = ["".join(sequence)]

        for e in sequence:
            if e == "or":
                or_groups.append([])
            else:
                or_groups[-1].append(e)
        if any(not g for g in or_groups):
            raise ValueError(
                f"Not able to parse type '{type_str}' used by {'.'.join(element_path)}"
            )

        new_elements: List[ast.AST] = []
        for group in or_groups:
            if len(group) == 1 and isinstance(group[0], str):
                new_elements.append(
                    concatenated_path_to_type(group[0], element_path, types_to_import)
                )
            elif (
                len(group) == 2
                and isinstance(group[0], str)
                and isinstance(group[1], list)
            ):
                new_elements.append(
                    ast.Subscript(
                        value=concatenated_path_to_type(
                            group[0], element_path, types_to_import
                        ),
                        slice=parse_sequence(group[1]),
                        ctx=ast.Load(),
                    )
                )
            else:
                raise ValueError(
                    f"Not able to parse type '{type_str}' used by {'.'.join(element_path)}"
                )
        return reduce(
            lambda left, right: ast.BinOp(left=left, op=ast.BitOr(), right=right),
            new_elements,
        )

    return parse_sequence(stack[0])


def concatenated_path_to_type(
    path: str, element_path: List[str], types_to_import: Set[str]
) -> ast.AST:
    parts = path.split(".")
    if any(not p for p in parts):
        raise ValueError(
            f"Not able to parse type '{path}' used by {'.'.join(element_path)}"
        )
    if len(parts) > 1:
        types_to_import.add(".".join(parts[:-1]))
    return path_to_type(*parts)


def build_doc_comment(doc: str) -> Optional[ast.Expr]:
    lines = [line.strip() for line in doc.split("\n")]
    clean_lines = []
    for line in lines:
        if line.startswith((":type", ":rtype")):
            continue
        clean_lines.append(line)
    text = "\n".join(clean_lines).strip()
    return ast.Expr(value=ast.Constant(text)) if text else None


def format_with_ruff(file: str) -> None:
    subprocess.check_call(["python", "-m", "ruff", "format", file])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract Python type stub from a python module."
    )
    parser.add_argument(
        "module_name", help="Name of the Python module for which generate stubs"
    )
    parser.add_argument(
        "out",
        help="Name of the Python stub file to write to",
        type=argparse.FileType("wt"),
    )
    parser.add_argument(
        "--ruff", help="Formats the generated stubs using Ruff", action="store_true"
    )
    args = parser.parse_args()
    stub_content = ast.unparse(module_stubs(importlib.import_module(args.module_name)))
    args.out.write(stub_content)
    if args.ruff:
        format_with_ruff(args.out.name)
