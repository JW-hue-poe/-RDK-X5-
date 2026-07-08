import inspect

path = inspect.__file__
with open(path, 'r') as f:
    lines = f.readlines()

new_lines = []
i = 0
while i < len(lines):
    new_lines.append(lines[i])

    # 修复 _signature_from_function
    if 'def _signature_from_function(' in lines[i]:
        # 跳过函数定义行和 docstring
        i += 1
        new_lines.append(lines[i])  # 第二行定义参数
        i += 1
        # docstring 可能是三行或一行
        while i < len(lines) and '"\'\'' not in lines[i] and not lines[i].strip().startswith('if'):
            new_lines.append(lines[i])
            i += 1
        # 在第一个非 docstring/空行之前插入初始化
        # 找到第一个缩进的代码行
        while i < len(lines) and (lines[i].strip() == '' or lines[i].lstrip().startswith('#')):
            new_lines.append(lines[i])
            i += 1
        if i < len(lines):
            new_lines.append('    keyword_only_count = 0\n')
            new_lines.append(lines[i])
            i += 1
        continue

    # 修复 getdoc
    if 'def getdoc(object):' in lines[i]:
        i += 1
        # 跳过 docstring（多行）
        in_docstring = False
        while i < len(lines):
            new_lines.append(lines[i])
            if lines[i].strip().startswith('"\'\''):
                in_docstring = not in_docstring
                if not in_docstring and lines[i].strip().count('"\'\'') >= 2:
                    pass
            elif not in_docstring and lines[i].strip() == 'try:':
                new_lines.insert(-1, '    doc = None\n')
                i += 1
                break
            i += 1
        continue

    i += 1

with open(path, 'w') as f:
    f.writelines(new_lines)
print('patched', path)
