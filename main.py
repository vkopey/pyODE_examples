import subprocess
import multiprocessing
import numpy as np
from scipy.optimize import differential_evolution

def f(x):
    p='d:\\Portable\\PortablePython27VTK_ODE\\App\\Python\\'
    command = [p+'python.exe', p+'ode4w_viz_example2.py']
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = process.communicate(input=np.array2string(x))
    return float(stdout)

if __name__=='__main__':
    #print(f(np.array([0.1, 0.5, 0.3])))

    multiprocessing.freeze_support()
    bounds = [(0.1, 0.2), (0.4, 0.5), (0.3, 0.4)]
    result = differential_evolution(f, bounds, maxiter=100, polish=False, workers=-1)
    print(result.x)
    print(result.fun)