import inspect

path = inspect.__file__
with open(path, 'r') as f:
    s = f.read()

# 修复 _signature_from_function：在 if isinstance(func, _NonUserDefinedCallables) 前插入初始化
old1 = '''    if isinstance(func, _NonUserDefinedCallables):
        # Built-in functions, methods and classes
        return cls.from_builtin(func,
                                skip_bound_arg=skip_bound_arg)
'''
new1 = '''    keyword_only_count = 0
    if isinstance(func, _NonUserDefinedCallables):
        # Built-in functions, methods and classes
        return cls.from_builtin(func,
                                skip_bound_arg=skip_bound_arg)
'''
if old1 in s:
    s = s.replace(old1, new1)
    print('patched _signature_from_function')
else:
    print('WARNING: _signature_from_function pattern not found')

# 修复 getdoc：在 try: 前插入 doc = None
old2 = '''    try:
        doc = object.__doc__
    except AttributeError:
        return None
'''
new2 = '''    doc = None
    try:
        doc = object.__doc__
    except AttributeError:
        return None
'''
if old2 in s:
    s = s.replace(old2, new2)
    print('patched getdoc')
else:
    print('WARNING: getdoc pattern not found')

with open(path, 'w') as f:
    f.write(s)
print('done:', path)
