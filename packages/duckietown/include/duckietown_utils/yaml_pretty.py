

# XXX: does not represent None as null, rather as '...\n'
def yaml_load(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    try:
        l = yaml.load(s, Loader=yaml.RoundTripLoader)
    except:
        l = yaml.load(s, Loader=yaml.UnsafeLoader)
    return l


def yaml_load_plain(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    l = yaml.load(s, Loader=yaml.UnsafeLoader)
    return l


def yaml_dump(s):
    from ruamel import yaml
    res = yaml.dump(s, Dumper=yaml.RoundTripDumper, allow_unicode=False)
    return res


def yaml_dump_pretty(ob):
    from ruamel import yaml
    return yaml.dump(ob, Dumper=yaml.RoundTripDumper)
