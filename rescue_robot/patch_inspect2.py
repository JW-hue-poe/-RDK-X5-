import inspect

path = inspect.__file__
with open(path, 'r') as f:
    lines = f.readlines()


def find_function_lines(lines, func_name):
    """找到函数定义的起始行和函数体第一行（跳过 docstring 后的第一行代码）"""
    start = None
    for i, line in enumerate(lines):
        if line.startswith('def ') and func_name + '(' in line:
            start = i
            break
    if start is None:
        return None, None

    # 找到函数体第一行（有缩进的非空行，跳过 docstring）
    # docstring 从第一个有4空格的行开始，如果是三引号则跳过到结束
    i = start + 1
    # 先跳过函数定义的第二行（如果参数在第二行）
    while i < len(lines) and (lines[i].strip() == '' or lines[i].strip().startswith('#')):
        i += 1
    # 现在可能在 docstring 开始
    if i < len(lines):
        stripped = lines[i].strip()
        if stripped.startswith('"""') or stripped.startswith("'''"):
            # 单行 docstring
            if stripped.count('"""') >= 2 or stripped.count("'''") >= 2:
                i += 1
            else:
                # 多行 docstring
                quote = '"""' if stripped.startswith('"""') else "'''"
                i += 1
                while i < len(lines) and quote not in lines[i]:
                    i += 1
                i += 1

    # 跳过空行和注释
    while i < len(lines) and (lines[i].strip() == '' or lines[i].strip().startswith('#')):
        i += 1

    return start, i


def find_try_line(lines, func_name):
    """找到函数内第一个 try: 的行号"""
    start, body_start = find_function_lines(lines, func_name)
    if body_start is None:
        return None
    for i in range(body_start, len(lines)):
        if lines[i].strip() == 'try:':
            return i
    return None


# 修复 _signature_from_function
_, body_start = find_function_lines(lines, '_signature_from_function')
if body_start is not None:
    lines.insert(body_start, '    keyword_only_count = 0\n')
    print('inserted keyword_only_count = 0 at line', body_start + 1)
else:
    print('WARNING: _signature_from_function not found')

# 修复 getdoc
try_line = find_try_line(lines, 'getdoc')
if try_line is not None:
    lines.insert(try_line, '    doc = None\n')
    print('inserted doc = None at line', try_line + 1)
else:
    print('WARNING: getdoc try not found')

with open(path, 'w') as f:
    f.writelines(lines)
print('done:', path)
