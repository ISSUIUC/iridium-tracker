a = '24f8e817066669cbcc010c139d19e917785b69cbd30013133c1fe917785969cba7001a13361ae9170b5c69cbd600211300'
out = "{"

for i in range(0, len(a), 2):
    out += "0x"
    out += a[i:i+2]
    out += ", "

out += "}"

print(out)
