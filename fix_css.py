path = r'c:\code2\AMR2.0\src\components\TaskManager\TaskManager.jsx'
with open(path, 'r', encoding='utf-8') as f:
    content = f.read()
old = "rgba(5b, 130, 246, 0.2)"
new = "rgba(91, 130, 246, 0.2)"
content = content.replace(old, new)
with open(path, 'w', encoding='utf-8') as f:
    f.write(content)
print('Fixed:', old, '->', new)
