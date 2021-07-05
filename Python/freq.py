#%% 
from sympy import symbols, diff, cos, sqrt, pi
import numpy as np

x = symbols('x')
f = cos(2*pi*x) - 1 + sqrt(cos(2*pi*x)**2 - 4 * cos(2*pi*x) + 3)

def taylor(x, a, f, n):
    p = np.empty((n,))
    df = f
    denom = 1
    for i in range(n):
        p[i] = df.evalf(subs={x: a}) * denom
        df = diff(df, x)
        denom /= i + 1
    return p

a = 0.25
p = taylor(x, a, f, 10)

print(p)

# %%

def horner(x, a, p):
    xa = x-a
    r = xa + p[-1] - xa # I'm sorry :(
    for fac in reversed(p[:-1]):
        r = r * xa + fac
    return r

def horner_impl(xa, p, i, t):
    if i == 0:
        return p[0] + xa * t
    else:
        return horner_impl(xa, p, i-1, p[i] + xa * t)

def horner_recursive(x, a, p):
    xa = x-a
    i = len(p)-1
    return horner_impl(xa, p, i-1, xa + p[i] - xa)

hh = horner(0.5, a, p)
rh = horner_recursive(0.5, a, p)
rr = f.evalf(subs={x: 0.5})

print(f'{hh=}')
print(f'{rh=}')
print(f'{rr=}')


# %%
import matplotlib.pyplot as plt

plt.figure()
xs = np.linspace(0, 0.5, 256)
plt.plot(xs, [f.evalf(subs={x: xxs}) for xxs in xs], label='f')
for i in range(1, len(p) + 1):
    plt.plot(xs, horner(xs, a, p[:i]), label=str(i))
plt.legend()
plt.show()
# %%

np.set_printoptions(precision=16, floatmode='fixed')
print(p)
', '.join(map(lambda x: np.format_float_scientific(x, precision=16, sign=True), p))

# %%

r = 0.5
for _ in range(10000): r = f.evalf(subs={x: r}) + .01
print(r*1e7)
# %%

horner(0.04612, a, p)
# %%
f.evalf(subs={x: 0.04612})
# %%
