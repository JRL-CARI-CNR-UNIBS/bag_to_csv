#!/usr/bin/env python3

import os
import importlib
import pkgutil
import yaml
import argparse

from jinja2 import Environment, FileSystemLoader

from rosidl_parser.definition import NamespacedType, AbstractSequence

# Discover all message classes in a given Python package
def discover_msgs(pkg_name):
    """
    Given a package name (e.g. 'std_msgs.msg'), import it and
    find all message classes within its submodules.
    Returns a list of tuples: (package_name, message_class_name).
    """
    pkg = importlib.import_module(pkg_name)
    msgs = []
    # Iterate over all immediate submodules of the package
    for _, module_name, is_pkg in pkgutil.iter_modules(pkg.__path__):
        if is_pkg:
            # skip directories/packages
            continue
        # import the module, e.g. std_msgs.msg.bool
        m = importlib.import_module(f"{pkg_name}.{module_name}")
        # inspect all attributes in the module
        for attr in dir(m):
            obj = getattr(m, attr)
            # identify ROS message classes by the presence of these attributes
            if (
                isinstance(obj, type)
                and hasattr(obj, '_fields_and_field_types')
                and hasattr(obj, 'SLOT_TYPES')
            ):
                msgs.append((pkg_name, attr))
    return msgs

def import_msg_class(nt: NamespacedType):
    """
    Given a NamespacedType (with namespaces like ['std_msgs','msg'] and name 'Header'),
    import the appropriate Python module and return the message class.
    """
    module_name = '.'.join(nt.namespaces)      # e.g. "std_msgs.msg"
    mod = importlib.import_module(module_name)
    return getattr(mod, nt.name)               # return the class, e.g. Header

def get_fields(msg_cls, parent_attrs=None):
    """
    Recursively walk through msg_cls, returning a list of dicts:
      - {'var': 'foo_bar', 'attr': ['foo','bar']}
      - {'var': 'baz', 'expr': 'msg.baz.sec + msg.baz.nanosec * 1e-9'}
    Special-cases any std_msgs/msg/Header → emits <prefix>_time and <prefix>_frame_id.
    """
    parent_attrs = parent_attrs or []  # track attribute path from the root message
    fields = []

    # helper to build the 'msg.xxx.yyy' prefix string from a list of attrs
    def msg_prefix(path):
        return 'msg' + ''.join(f'.{p}' for p in path)

    # iterate in declaration order
    for idx, field_name in enumerate(msg_cls._fields_and_field_types):
        slot_type = msg_cls.SLOT_TYPES[idx]

        # ─── Special-case Header ────────────────────────────────────────────────
        # If this field is a std_msgs/msg/Header, generate a combined time field
        if (
            isinstance(slot_type, NamespacedType)
            and slot_type.namespaces == ['std_msgs', 'msg']
            and slot_type.name == 'Header'
        ):
            path = parent_attrs + [field_name]
            # create a synthetic 'time' variable combining sec + nanosec
            var_time = '_'.join(path + ['time'])
            expr_time = (
                f"{msg_prefix(path)}.stamp.sec"
                f" + {msg_prefix(path)}.stamp.nanosec * 1e-9"
            )
            fields.append({'var': var_time, 'expr': expr_time})
            # also extract frame_id normally
            fields.append({
                'var': '_'.join(path + ['frame_id']),
                'attr': path + ['frame_id']
            })
            # skip further recursion into this Header
            continue

        # ─── Nested message → recurse ───────────────────────────────────────────
        if isinstance(slot_type, NamespacedType):
            nested_cls = import_msg_class(slot_type)
            # recurse with the updated attribute path
            fields.extend(
                get_fields(nested_cls, parent_attrs + [field_name])
            )
            continue

        # ─── Sequence of nested messages → recurse ─────────────────────────────
        if (
            isinstance(slot_type, AbstractSequence)
            and isinstance(slot_type.value_type, NamespacedType)
        ):
            nested_cls = import_msg_class(slot_type.value_type)
            fields.extend(
                get_fields(nested_cls, parent_attrs + [field_name])
            )
            continue

        # ─── Primitive or primitive sequence → leaf node ──────────────────────
        # build a flattened field entry for simple types
        attrs = parent_attrs + [field_name]
        fields.append({
            'var': '_'.join(attrs),
            'attr': attrs
        })

    return fields


def main():
    # list of ROS Python packages to scan for message types
    parser = argparse.ArgumentParser(
        description="Generate ROS2 message extractors based on a YAML list of packages"
    )
    parser.add_argument(
        '-c', '--config',
        help='Path to the YAML config file listing packages'
    )
    args = parser.parse_args()

    pkgs_prefix = []
    if(args.config):
      with open(args.config, 'r') as f:
        config_yml = yaml.safe_load(f)
        pkgs_prefix = config_yml['packages']
        if not pkgs_prefix:
            raise RuntimeError(f"No packages specified in {args.config}")
    else:
      pkgs_prefix = ['std_msgs', 'geometry_msgs', 'nav_msgs', 'nav2_msgs', 'sensor_msgs']
    pkgs = [f'{s}.msg' for s in pkgs_prefix]
    print(pkgs)
    print(args.config)

    # output directory for generated extractor files
    out_dir = os.path.join('common_extractors')
    os.makedirs(out_dir, exist_ok=True)

    # configure Jinja2 environment for template rendering
    env = Environment(
        loader=FileSystemLoader('templates'),
        trim_blocks=False,
        lstrip_blocks=False,
    )
    template = env.get_template('extractor_template.j2')

    # iterate over each package and its discovered message types
    for pkg_name in pkgs:
        for pkg, msg_name in discover_msgs(pkg_name):
            # dynamically import the module and get the message class
            mod = importlib.import_module(pkg)
            msg_cls = getattr(mod, msg_name)

            # generate the list of fields (with flattening and Header handling)
            fields = get_fields(msg_cls)
            class_name = f"{msg_name}Extractor"
            # ROS type identifier string (used by InfoExtractor)
            type_str = f"{pkg.replace('.', '/')}/{msg_name}"

            # render the Jinja2 template with our context
            code = template.render(
                import_pkg=pkg,
                msg_name=msg_name,
                class_name=class_name,
                type_str=type_str,
                fields=fields,
            )

            # write out the generated Python file
            fname = os.path.join(out_dir, f"{msg_name.lower()}_extractor.py")
            with open(fname, 'w') as f:
                f.write(code)
            print(f"✓ Generated {fname}")


if __name__ == '__main__':
    main()
