a = 'f806abfb177a2950cbe70003043fe6fb17ac2950cbe200040c491dfc17952950cbe00004143a54fc17472950cbdf0004'
out = "{"

for i in range(0, len(a), 2):
    out += "0x"
    out += a[i:i+2]
    out += ", "

out += "}"

print(out)
