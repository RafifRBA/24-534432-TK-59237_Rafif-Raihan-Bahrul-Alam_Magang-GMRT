import numpy as np
import math
import matplotlib.pyplot as plt

def rotation(thetadeg):
    theta = math.radians(thetadeg)
    c = math.cos(theta)
    s = math.sin(theta)
    matrix = np.array([[c, -s, 0, 0],
                       [s, c, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]], dtype=float)
    return matrix

    
def translasi(length):
    matrix = np.array([[1, 0, 0, length],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]], dtype=float)
    return matrix


def dof2_fk(theta1, length1, theta2, length2):
    rot1 = rotation(theta1)
    tran1 = translasi(length1)
    rot2 = rotation(theta2)
    tran2 = translasi(length2)

    mult1 = rot1 @ tran1
    mult2 = rot2 @ tran2
    mult = mult1 @ mult2

    cox = np.array([0, 0, 0, 1], dtype=float)               # mencatat base
    fem = mult1 @ cox                                       # mencatat titik akhir femur
    tib = mult @ cox                                        # mencatat titik akhir tibia

    titik_penting = np.vstack([cox[:2], fem[:2], tib[:2]])  # ambil masing2 titik x dan y
   
    return titik_penting


def dof3_fk(theta1, length1, theta2, length2, theta3, length3):
    rot1 = rotation(theta1)
    tran1 = translasi(length1)
    rot2 = rotation(theta2)
    tran2 = translasi(length2)
    rot3 = rotation(theta3)
    tran3 = translasi(length3)

    mult1 = rot1 @ tran1
    mult2 = rot2 @ tran2
    mult3 = rot3 @ tran3
    mult = mult1 @ mult2 @ mult3

    base = np.array([0, 0, 0, 1], dtype=float)
    joint1 = mult1 @ base
    joint2 = (mult1 @ mult2) @ base
    joint3 = mult @ base

    titik_penting = np.vstack([base[:2], joint1[:2], joint2[:2], joint3[:2]])

    return titik_penting


def inverseK(length1, length2, x, y):
    r = x**2 + y**2
    thcos = (r - (length1**2 + length2**2))/(2.0*length1*length2)
    theta2 = math.acos(thcos)
    k1 = length2 * math.sin(theta2)
    k2 = length1 + (length2 * math.cos(theta2))
    theta1 = math.atan2(y, x) - math.atan2(k1, k2)
    th1 = math.degrees(theta1)
    th2 = math.degrees(theta2)

    return th1, th2

def plotting_2d(joints, title):
    fig = plt.figure()
    x, y = joints[:, 0], joints[:, 1]
    plt.plot(x, y, marker='o', markerfacecolor='red')
    plt.title(title)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    return fig


def main():
    pil = int(input("Pilih Mau Apa? \n1. 2dof\n2. 3dof\n3. inverse kinematics untuk 2 dof\nKetik 1, 2, atau 3\n>> "))
    if pil == 1:
       th1 = float(input("Masukkan theta 1: "))
       L1 = float(input("Masukkan length 1: "))
       th2 = float(input("Masukkan theta 2: "))
       L2 = float(input("Masukkan length 2: "))
       joints = dof2_fk(th1, L1, th2, L2)
       print(f"Hasil koordinat akhir: x = {joints[2][0]:.3f} , y = {joints[2][1]:.3f}")
       plotting_2d(joints, title=f"2-DoF: th1={th1}°, L1={L1}, th2={th2}°, L2={L2}")
       plt.show()
    
    elif pil == 2:
        th1 = float(input("Masukkan theta 1: "))
        L1 = float(input("Masukkan length 1: "))
        th2 = float(input("Masukkan theta 2: "))
        L2 = float(input("Masukkan length 2: "))
        th3 = float(input("Masukkan theta3: "))
        L3 = float(input("Masukkan length 3: "))
        joints = dof3_fk(th1, L1, th2, L2, th3, L3)
        print(f"Hasil koordinat akhir: x = {joints[3][0]:.3f} , y = {joints[3][1]:.3f}")
        plotting_2d(joints, title=f"3-DoF: th1={th1}°, L1={L1}, th2={th2}°, L2={L2}, th3={th3}°, L3={L3}")
        plt.show()
    
    elif pil == 3:
        x = float(input("Masukkan nilai x posisi akhir: "))
        y = float(input("Masukkan nilai y posisi akhir: "))
        l1 = float(input("Masukkan nilai L1: "))
        l2 = float(input("Masukkan nilai L2: "))
        th1, th2 = inverseK(l1, l2, x, y)
        print(f"Hasil theta1 dan theta2 dalam degree: th1 = {th1:.2f} , th2 = {th2:.2f}")
        joints = dof2_fk(th1, l1, th2, l2)
        plotting_2d(joints, title=f"Inverse Kinematics: th1={th1:.2f}°, th2={th2:.2f}°")
        plt.show()

    
    else:
        print("Mode tidak dikenali. Gunakan '1', '2', atau '3' untuk memilih opsi.")
       

if __name__ == "__main__":
    main()