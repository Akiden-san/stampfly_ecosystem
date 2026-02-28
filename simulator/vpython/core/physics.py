# MIT License
# 
# Copyright (c) 2025 Kouhei Ito
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



import math
import numpy as np


def _cross3(a, b):
    """Manual cross product for 3x1 column vectors (avoids np.cross overhead).
    3x1列ベクトル用の手動外積計算（np.crossのオーバーヘッドを回避）"""
    return np.array([
        [a[1][0]*b[2][0] - a[2][0]*b[1][0]],
        [a[2][0]*b[0][0] - a[0][0]*b[2][0]],
        [a[0][0]*b[1][0] - a[1][0]*b[0][0]]
    ])


#DCM(Direct Cosine Matrix) is transform from body frame to inertial frame.

class rigidbody():
    def __init__(self, mass=1.0, inersia=np.eye(3), position=[[0.0],[0.0],[0.0]], velocity=[[0.0],[0.0],[0.0]], pqr=[[0.0],[0.0],[0.0]], euler=[[0.0],[0.0],[0.0]]):
        #DCM(Direct Cosine Matrix) from body frame to inertial frame:
        self.mass = mass
        self.inertia = inersia#np.array()#np.eye(3)
        # Pre-compute inverse inertia matrix (constant, avoids 8000 inv/s)
        # 慣性逆行列を事前計算（定数なので毎ステップの計算を回避）
        self.inertia_inv = np.linalg.inv(np.array(inersia))
        self.euler = np.array(euler)
        self.quat = self.euler2quat(euler)
        self.DCM = self.quat_dcm(self.quat)
        self.position = np.array(position)#np.zeros((3,1))
        self.velocity = np.array(velocity)#
        self.pqr = np.array(pqr)
        self.uvw = self.velocity2uvw(velocity, self.DCM)

        # Scalar state for fast path (no numpy overhead)
        # 高速パス用スカラー状態（numpyオーバーヘッド回避）
        self.s_u = float(self.uvw[0][0])
        self.s_v = float(self.uvw[1][0])
        self.s_w = float(self.uvw[2][0])
        self.s_p = float(self.pqr[0][0])
        self.s_q = float(self.pqr[1][0])
        self.s_r = float(self.pqr[2][0])
        self.s_q0 = float(self.quat[0][0])
        self.s_q1 = float(self.quat[1][0])
        self.s_q2 = float(self.quat[2][0])
        self.s_q3 = float(self.quat[3][0])
        self.s_x = float(self.position[0][0])
        self.s_y = float(self.position[1][0])
        self.s_z = float(self.position[2][0])

        # Diagonal inertia scalar cache (avoids matrix ops)
        # 対角慣性行列のスカラーキャッシュ（行列演算を回避）
        self._Ixx = float(inersia[0][0])
        self._Iyy = float(inersia[1][1])
        self._Izz = float(inersia[2][2])
        self._Ixx_inv = 1.0 / self._Ixx
        self._Iyy_inv = 1.0 / self._Iyy
        self._Izz_inv = 1.0 / self._Izz
        self._Izz_minus_Iyy = self._Izz - self._Iyy
        self._Ixx_minus_Izz = self._Ixx - self._Izz
        self._Iyy_minus_Ixx = self._Iyy - self._Ixx

        # Inverse mass (scalar, constant)
        # 逆質量（スカラー、定数）
        self._m_inv = 1.0 / self.mass

        # DCM scalar cache (initialized from current DCM)
        # DCMスカラーキャッシュ（現在のDCMから初期化）
        self._dcm00 = float(self.DCM[0][0]); self._dcm01 = float(self.DCM[0][1]); self._dcm02 = float(self.DCM[0][2])
        self._dcm10 = float(self.DCM[1][0]); self._dcm11 = float(self.DCM[1][1]); self._dcm12 = float(self.DCM[1][2])
        self._dcm20 = float(self.DCM[2][0]); self._dcm21 = float(self.DCM[2][1]); self._dcm22 = float(self.DCM[2][2])

    def quat_dcm(self, quat):
        q0 = quat[0][0]
        q1 = quat[1][0]
        q2 = quat[2][0]
        q3 = quat[3][0]
        DCM = np.array([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2) ], 
                        [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1) ], 
                        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2 ]])
        return DCM

    def euler_dcm(self, euler):
        phi = euler[0][0]
        tht = euler[1][0]
        psi = euler[2][0]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        s_tht = np.sin(tht)
        c_tht = np.cos(tht)
        s_psi = np.sin(psi)
        c_psi = np.cos(psi)
        DCM = np.array([[c_tht*c_psi, s_phi*s_tht*c_psi - c_phi*s_psi, c_phi*s_tht*c_psi + s_phi*s_psi], 
                        [c_tht*s_psi, s_phi*s_tht*s_psi + c_phi*c_psi, c_phi*s_tht*s_psi - s_phi*c_psi], 
                        [-s_tht, s_phi*c_tht, c_phi*c_tht]])
        return DCM
        
    def velocity(self, uvw, DCM):
        return DCM @ uvw

    def euler_dot(self, euler, pqr):
        phi = euler[0][0]
        tht = euler[1][0]
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        t_phi = np.tan(phi)
        c_tht = np.cos(tht)    
        euler_dot = np.array([[1, s_phi*t_phi, c_phi*t_phi], 
                              [0, c_phi, -s_phi], 
                              [0, s_phi/c_tht, c_phi/c_tht]]) @ pqr
        return euler_dot

    def quat_dot(self, quat, pqr):
        p = pqr[0][0]
        q = pqr[1][0]
        r = pqr[2][0]
        quat_dot = 0.5*np.array([[0, -p, -q, -r], 
                                 [p, 0, r, -q], 
                                 [q, -r, 0, p], 
                                 [r, q, -p, 0]]) @ quat
        return quat_dot
    
    def uvw_dot(self, uvw, pqr, force):
        #m ([duvw/dt] + omaga x uvw) = force
        #[duvw/dt] = force/m - (omaga x uvw)
        force = np.array(force)
        uvw_dot = force/self.mass - _cross3(pqr, uvw)
        return uvw_dot

    def pqr_dot(self, pqr, torque):
        #dH/dt = torque
        #H = I @ pqr
        #dH/dt = [dH/dt]+pqr x H
        #[dI@pqr/dt] + pqr x (I @ pqr) = torque
        #I @ [dpqr/dt] + pqr x (I @ pqr) = torque
        #I @ [dpqr/dt] = torque - pqr x (I @ pqr)
        #dpqr/dt = I^-1 @ (torque - pqr x (I @ pqr))
        #pqr_dot = I^-1 @ (torque - pqr x (I @ pqr))
        torque = np.array(torque)
        pqr_dot = self.inertia_inv @ (torque - _cross3(pqr, self.inertia @ pqr))
        return pqr_dot
    
    def position_dot(self, uvw, quat):
        #dposition/dt = velocity
        #velocity = DCM @ uvw
        #dposition/dt = DCM @ uvw
        dcm = self.quat_dcm(quat)
        position_dot = dcm @ uvw
        return position_dot

    def uvw2velocity(self, uvw, DCM):
        velocity = DCM @ uvw
        return velocity
    
    def velocity2uvw(self, velocity, DCM):
        uvw = DCM.T @ velocity
        return uvw
    
    def euler2quat(self, euler):
        euler = np.array(euler)
        phi = euler[0][0]
        tht = euler[1][0]
        psi = euler[2][0]
        s_phi = np.sin(phi/2)
        c_phi = np.cos(phi/2)
        s_tht = np.sin(tht/2)
        c_tht = np.cos(tht/2)
        s_psi = np.sin(psi/2)
        c_psi = np.cos(psi/2)
        quat = np.array([[c_phi*c_tht*c_psi + s_phi*s_tht*s_psi], 
                         [s_phi*c_tht*c_psi - c_phi*s_tht*s_psi], 
                         [c_phi*s_tht*c_psi + s_phi*c_tht*s_psi], 
                         [c_phi*c_tht*s_psi - s_phi*s_tht*c_psi]])
        return quat
    
    def quat2euler(self, quat):
        q0=quat[0][0]
        q1=quat[1][0]
        q2=quat[2][0]
        q3=quat[3][0]

        asin = -2*(q1*q3 - q0*q2)
        if asin > 1.0:
            asin = 1.0 
        elif asin < -1.0:
            asin = -1.0

        euler = np.array([[np.arctan2(2*(q0*q1 + q2*q3), q0**2 - q1**2 - q2**2 + q3**2)], 
                              [np.arcsin(asin)], 
                              [np.arctan2(2*(q0*q3 + q1*q2), q0**2 + q1**2 - q2**2 - q3**2)]])
        return euler
    
    def normalize_quat(self):
        self.quat = self.quat/np.linalg.norm(self.quat)

    def set_pqr(self, pqr):
        self.pqr = np.array(pqr)
        self.s_p = float(self.pqr[0][0])
        self.s_q = float(self.pqr[1][0])
        self.s_r = float(self.pqr[2][0])

    def set_uvw(self, uvw):
        self.uvw = np.array(uvw)
        self.s_u = float(self.uvw[0][0])
        self.s_v = float(self.uvw[1][0])
        self.s_w = float(self.uvw[2][0])

    def set_quat(self, quat):
        self.quat = np.array(quat)
        self.dcm = self.quat_dcm(self.quat)
        self.euler = self.quat2euler(self.quat)
        self.s_q0 = float(self.quat[0][0])
        self.s_q1 = float(self.quat[1][0])
        self.s_q2 = float(self.quat[2][0])
        self.s_q3 = float(self.quat[3][0])
        self._update_dcm_cache(self.dcm)

    def set_euler(self, euler):
        self.euler = np.array(euler)
        self.quat = self.euler2quat(self.euler)
        self.dcm = self.euler_dcm(self.euler)
        self.s_q0 = float(self.quat[0][0])
        self.s_q1 = float(self.quat[1][0])
        self.s_q2 = float(self.quat[2][0])
        self.s_q3 = float(self.quat[3][0])
        self._update_dcm_cache(self.dcm)

    def set_position(self, position):
        self.position = np.array(position)
        self.s_x = float(self.position[0][0])
        self.s_y = float(self.position[1][0])
        self.s_z = float(self.position[2][0])

    def _update_dcm_cache(self, dcm):
        """Update scalar DCM cache from numpy DCM matrix.
        numpy DCM行列からスカラーDCMキャッシュを更新"""
        self._dcm00 = float(dcm[0][0]); self._dcm01 = float(dcm[0][1]); self._dcm02 = float(dcm[0][2])
        self._dcm10 = float(dcm[1][0]); self._dcm11 = float(dcm[1][1]); self._dcm12 = float(dcm[1][2])
        self._dcm20 = float(dcm[2][0]); self._dcm21 = float(dcm[2][1]); self._dcm22 = float(dcm[2][2])

    def step(self, force, torque, dt):
        #RK4で剛体の運動方程式を解くとともに,グローバル座標系の速度,位置,DCMを更新する
        #1. k1を求める
        k1_uvw = self.uvw_dot(self.uvw, self.pqr, force)
        k1_pqr = self.pqr_dot(self.pqr, torque)
        k1_quat = self.quat_dot(self.quat, self.pqr)
        k1_position = self.position_dot(self.uvw, self.quat)
        #2. k2を求める
        k2_uvw = self.uvw_dot(self.uvw + 0.5*dt*k1_uvw, self.pqr + 0.5*dt*k1_pqr, force)
        k2_pqr = self.pqr_dot(self.pqr + 0.5*dt*k1_pqr, torque)
        k2_quat = self.quat_dot(self.quat + 0.5*dt*k1_quat, self.pqr + 0.5*dt*k1_pqr)
        k2_position = self.position_dot(self.uvw + 0.5*dt*k1_uvw, self.quat + 0.5*dt*k1_quat)
        #3. k3を求める
        k3_uvw = self.uvw_dot(self.uvw + 0.5*dt*k2_uvw, self.pqr + 0.5*dt*k2_pqr, force)
        k3_pqr = self.pqr_dot(self.pqr + 0.5*dt*k2_pqr, torque)
        k3_quat = self.quat_dot(self.quat + 0.5*dt*k2_quat, self.pqr + 0.5*dt*k2_pqr)
        k3_position = self.position_dot(self.uvw + 0.5*dt*k2_uvw, self.quat + 0.5*dt*k2_quat)
        #4. k4を求める
        k4_uvw = self.uvw_dot(self.uvw + dt*k3_uvw, self.pqr + dt*k3_pqr, force)
        k4_pqr = self.pqr_dot(self.pqr + dt*k3_pqr, torque)
        k4_quat = self.quat_dot(self.quat + dt*k3_quat, self.pqr + dt*k3_pqr)
        k4_position = self.position_dot(self.uvw + dt*k3_uvw, self.quat + dt*k3_quat)
        #5. 次の状態を求める
        self.uvw += dt*(k1_uvw + 2*k2_uvw + 2*k3_uvw + k4_uvw)/6
        self.pqr += dt*(k1_pqr + 2*k2_pqr + 2*k3_pqr + k4_pqr)/6
        self.quat += dt*(k1_quat + 2*k2_quat + 2*k3_quat + k4_quat)/6
        self.position += dt*(k1_position + 2*k2_position + 2*k3_position + k4_position)/6
        #6. quatを正規化する
        self.normalize_quat()
        #7. DCMを更新する
        self.DCM = self.quat_dcm(self.quat)
        #8. eulerを更新する
        self.euler = self.quat2euler(self.quat)
        #9. velocityを更新する
        self.velocity = self.uvw2velocity(self.uvw, self.DCM)

    def step_fast(self, fx, fy, fz, L, M, N, dt):
        """RK4 integration using pure Python scalars (no numpy overhead).
        純粋なPythonスカラーによるRK4積分（numpyオーバーヘッドなし）"""
        # Read state into locals (faster than self.s_* access)
        # ローカル変数に読み込み（self.s_*アクセスより高速）
        u = self.s_u; v = self.s_v; w = self.s_w
        p = self.s_p; q = self.s_q; r = self.s_r
        Q0 = self.s_q0; Q1 = self.s_q1; Q2 = self.s_q2; Q3 = self.s_q3
        x = self.s_x; y = self.s_y; z = self.s_z

        # Constants
        m_inv = self._m_inv
        Ixx_inv = self._Ixx_inv; Iyy_inv = self._Iyy_inv; Izz_inv = self._Izz_inv
        IzzIyy = self._Izz_minus_Iyy
        IxxIzz = self._Ixx_minus_Izz
        IyyIxx = self._Iyy_minus_Ixx
        hdt = 0.5 * dt

        # ===== Stage 1 (k1) =====
        # uvw_dot = force/mass - cross(pqr, uvw)
        k1_du = fx * m_inv - (q * w - r * v)
        k1_dv = fy * m_inv - (r * u - p * w)
        k1_dw = fz * m_inv - (p * v - q * u)
        # pqr_dot = I_inv @ (torque - cross(pqr, I @ pqr))
        k1_dp = (L - q * r * IzzIyy) * Ixx_inv
        k1_dq = (M - r * p * IxxIzz) * Iyy_inv
        k1_dr = (N - p * q * IyyIxx) * Izz_inv
        # quat_dot = 0.5 * Omega(pqr) @ quat
        k1_dQ0 = 0.5 * (-p * Q1 - q * Q2 - r * Q3)
        k1_dQ1 = 0.5 * ( p * Q0 + r * Q2 - q * Q3)
        k1_dQ2 = 0.5 * ( q * Q0 - r * Q1 + p * Q3)
        k1_dQ3 = 0.5 * ( r * Q0 + q * Q1 - p * Q2)
        # DCM from current quaternion for position_dot
        d00 = Q0*Q0 + Q1*Q1 - Q2*Q2 - Q3*Q3
        d01 = 2.0*(Q1*Q2 - Q0*Q3); d02 = 2.0*(Q1*Q3 + Q0*Q2)
        d10 = 2.0*(Q1*Q2 + Q0*Q3)
        d11 = Q0*Q0 - Q1*Q1 + Q2*Q2 - Q3*Q3
        d12 = 2.0*(Q2*Q3 - Q0*Q1)
        d20 = 2.0*(Q1*Q3 - Q0*Q2); d21 = 2.0*(Q2*Q3 + Q0*Q1)
        d22 = Q0*Q0 - Q1*Q1 - Q2*Q2 + Q3*Q3
        # position_dot = DCM @ uvw
        k1_dx = d00*u + d01*v + d02*w
        k1_dy = d10*u + d11*v + d12*w
        k1_dz = d20*u + d21*v + d22*w

        # ===== Stage 2 (k2) - state + 0.5*dt*k1 =====
        u2 = u + hdt*k1_du; v2 = v + hdt*k1_dv; w2 = w + hdt*k1_dw
        p2 = p + hdt*k1_dp; q2 = q + hdt*k1_dq; r2 = r + hdt*k1_dr
        Q0_2 = Q0 + hdt*k1_dQ0; Q1_2 = Q1 + hdt*k1_dQ1
        Q2_2 = Q2 + hdt*k1_dQ2; Q3_2 = Q3 + hdt*k1_dQ3
        k2_du = fx * m_inv - (q2 * w2 - r2 * v2)
        k2_dv = fy * m_inv - (r2 * u2 - p2 * w2)
        k2_dw = fz * m_inv - (p2 * v2 - q2 * u2)
        k2_dp = (L - q2 * r2 * IzzIyy) * Ixx_inv
        k2_dq = (M - r2 * p2 * IxxIzz) * Iyy_inv
        k2_dr = (N - p2 * q2 * IyyIxx) * Izz_inv
        k2_dQ0 = 0.5 * (-p2*Q1_2 - q2*Q2_2 - r2*Q3_2)
        k2_dQ1 = 0.5 * ( p2*Q0_2 + r2*Q2_2 - q2*Q3_2)
        k2_dQ2 = 0.5 * ( q2*Q0_2 - r2*Q1_2 + p2*Q3_2)
        k2_dQ3 = 0.5 * ( r2*Q0_2 + q2*Q1_2 - p2*Q2_2)
        d00 = Q0_2*Q0_2 + Q1_2*Q1_2 - Q2_2*Q2_2 - Q3_2*Q3_2
        d01 = 2.0*(Q1_2*Q2_2 - Q0_2*Q3_2); d02 = 2.0*(Q1_2*Q3_2 + Q0_2*Q2_2)
        d10 = 2.0*(Q1_2*Q2_2 + Q0_2*Q3_2)
        d11 = Q0_2*Q0_2 - Q1_2*Q1_2 + Q2_2*Q2_2 - Q3_2*Q3_2
        d12 = 2.0*(Q2_2*Q3_2 - Q0_2*Q1_2)
        d20 = 2.0*(Q1_2*Q3_2 - Q0_2*Q2_2); d21 = 2.0*(Q2_2*Q3_2 + Q0_2*Q1_2)
        d22 = Q0_2*Q0_2 - Q1_2*Q1_2 - Q2_2*Q2_2 + Q3_2*Q3_2
        k2_dx = d00*u2 + d01*v2 + d02*w2
        k2_dy = d10*u2 + d11*v2 + d12*w2
        k2_dz = d20*u2 + d21*v2 + d22*w2

        # ===== Stage 3 (k3) - state + 0.5*dt*k2 =====
        u3 = u + hdt*k2_du; v3 = v + hdt*k2_dv; w3 = w + hdt*k2_dw
        p3 = p + hdt*k2_dp; q3 = q + hdt*k2_dq; r3 = r + hdt*k2_dr
        Q0_3 = Q0 + hdt*k2_dQ0; Q1_3 = Q1 + hdt*k2_dQ1
        Q2_3 = Q2 + hdt*k2_dQ2; Q3_3 = Q3 + hdt*k2_dQ3
        k3_du = fx * m_inv - (q3 * w3 - r3 * v3)
        k3_dv = fy * m_inv - (r3 * u3 - p3 * w3)
        k3_dw = fz * m_inv - (p3 * v3 - q3 * u3)
        k3_dp = (L - q3 * r3 * IzzIyy) * Ixx_inv
        k3_dq = (M - r3 * p3 * IxxIzz) * Iyy_inv
        k3_dr = (N - p3 * q3 * IyyIxx) * Izz_inv
        k3_dQ0 = 0.5 * (-p3*Q1_3 - q3*Q2_3 - r3*Q3_3)
        k3_dQ1 = 0.5 * ( p3*Q0_3 + r3*Q2_3 - q3*Q3_3)
        k3_dQ2 = 0.5 * ( q3*Q0_3 - r3*Q1_3 + p3*Q3_3)
        k3_dQ3 = 0.5 * ( r3*Q0_3 + q3*Q1_3 - p3*Q2_3)
        d00 = Q0_3*Q0_3 + Q1_3*Q1_3 - Q2_3*Q2_3 - Q3_3*Q3_3
        d01 = 2.0*(Q1_3*Q2_3 - Q0_3*Q3_3); d02 = 2.0*(Q1_3*Q3_3 + Q0_3*Q2_3)
        d10 = 2.0*(Q1_3*Q2_3 + Q0_3*Q3_3)
        d11 = Q0_3*Q0_3 - Q1_3*Q1_3 + Q2_3*Q2_3 - Q3_3*Q3_3
        d12 = 2.0*(Q2_3*Q3_3 - Q0_3*Q1_3)
        d20 = 2.0*(Q1_3*Q3_3 - Q0_3*Q2_3); d21 = 2.0*(Q2_3*Q3_3 + Q0_3*Q1_3)
        d22 = Q0_3*Q0_3 - Q1_3*Q1_3 - Q2_3*Q2_3 + Q3_3*Q3_3
        k3_dx = d00*u3 + d01*v3 + d02*w3
        k3_dy = d10*u3 + d11*v3 + d12*w3
        k3_dz = d20*u3 + d21*v3 + d22*w3

        # ===== Stage 4 (k4) - state + dt*k3 =====
        u4 = u + dt*k3_du; v4 = v + dt*k3_dv; w4 = w + dt*k3_dw
        p4 = p + dt*k3_dp; q4 = q + dt*k3_dq; r4 = r + dt*k3_dr
        Q0_4 = Q0 + dt*k3_dQ0; Q1_4 = Q1 + dt*k3_dQ1
        Q2_4 = Q2 + dt*k3_dQ2; Q3_4 = Q3 + dt*k3_dQ3
        k4_du = fx * m_inv - (q4 * w4 - r4 * v4)
        k4_dv = fy * m_inv - (r4 * u4 - p4 * w4)
        k4_dw = fz * m_inv - (p4 * v4 - q4 * u4)
        k4_dp = (L - q4 * r4 * IzzIyy) * Ixx_inv
        k4_dq = (M - r4 * p4 * IxxIzz) * Iyy_inv
        k4_dr = (N - p4 * q4 * IyyIxx) * Izz_inv
        k4_dQ0 = 0.5 * (-p4*Q1_4 - q4*Q2_4 - r4*Q3_4)
        k4_dQ1 = 0.5 * ( p4*Q0_4 + r4*Q2_4 - q4*Q3_4)
        k4_dQ2 = 0.5 * ( q4*Q0_4 - r4*Q1_4 + p4*Q3_4)
        k4_dQ3 = 0.5 * ( r4*Q0_4 + q4*Q1_4 - p4*Q2_4)
        d00 = Q0_4*Q0_4 + Q1_4*Q1_4 - Q2_4*Q2_4 - Q3_4*Q3_4
        d01 = 2.0*(Q1_4*Q2_4 - Q0_4*Q3_4); d02 = 2.0*(Q1_4*Q3_4 + Q0_4*Q2_4)
        d10 = 2.0*(Q1_4*Q2_4 + Q0_4*Q3_4)
        d11 = Q0_4*Q0_4 - Q1_4*Q1_4 + Q2_4*Q2_4 - Q3_4*Q3_4
        d12 = 2.0*(Q2_4*Q3_4 - Q0_4*Q1_4)
        d20 = 2.0*(Q1_4*Q3_4 - Q0_4*Q2_4); d21 = 2.0*(Q2_4*Q3_4 + Q0_4*Q1_4)
        d22 = Q0_4*Q0_4 - Q1_4*Q1_4 - Q2_4*Q2_4 + Q3_4*Q3_4
        k4_dx = d00*u4 + d01*v4 + d02*w4
        k4_dy = d10*u4 + d11*v4 + d12*w4
        k4_dz = d20*u4 + d21*v4 + d22*w4

        # ===== State update: state += dt/6 * (k1 + 2*k2 + 2*k3 + k4) =====
        dt6 = dt / 6.0
        self.s_u = u + dt6 * (k1_du + 2.0*k2_du + 2.0*k3_du + k4_du)
        self.s_v = v + dt6 * (k1_dv + 2.0*k2_dv + 2.0*k3_dv + k4_dv)
        self.s_w = w + dt6 * (k1_dw + 2.0*k2_dw + 2.0*k3_dw + k4_dw)
        self.s_p = p + dt6 * (k1_dp + 2.0*k2_dp + 2.0*k3_dp + k4_dp)
        self.s_q = q + dt6 * (k1_dq + 2.0*k2_dq + 2.0*k3_dq + k4_dq)
        self.s_r = r + dt6 * (k1_dr + 2.0*k2_dr + 2.0*k3_dr + k4_dr)
        self.s_q0 = Q0 + dt6 * (k1_dQ0 + 2.0*k2_dQ0 + 2.0*k3_dQ0 + k4_dQ0)
        self.s_q1 = Q1 + dt6 * (k1_dQ1 + 2.0*k2_dQ1 + 2.0*k3_dQ1 + k4_dQ1)
        self.s_q2 = Q2 + dt6 * (k1_dQ2 + 2.0*k2_dQ2 + 2.0*k3_dQ2 + k4_dQ2)
        self.s_q3 = Q3 + dt6 * (k1_dQ3 + 2.0*k2_dQ3 + 2.0*k3_dQ3 + k4_dQ3)
        self.s_x = x + dt6 * (k1_dx + 2.0*k2_dx + 2.0*k3_dx + k4_dx)
        self.s_y = y + dt6 * (k1_dy + 2.0*k2_dy + 2.0*k3_dy + k4_dy)
        self.s_z = z + dt6 * (k1_dz + 2.0*k2_dz + 2.0*k3_dz + k4_dz)

        # Normalize quaternion
        # クォータニオン正規化
        qn = math.sqrt(self.s_q0*self.s_q0 + self.s_q1*self.s_q1
                        + self.s_q2*self.s_q2 + self.s_q3*self.s_q3)
        if qn > 0.0:
            qn_inv = 1.0 / qn
            self.s_q0 *= qn_inv; self.s_q1 *= qn_inv
            self.s_q2 *= qn_inv; self.s_q3 *= qn_inv

        # Compute final DCM from updated quaternion and cache
        # 更新されたクォータニオンから最終DCMを計算しキャッシュ
        Q0 = self.s_q0; Q1 = self.s_q1; Q2 = self.s_q2; Q3 = self.s_q3
        d00 = Q0*Q0 + Q1*Q1 - Q2*Q2 - Q3*Q3
        d01 = 2.0*(Q1*Q2 - Q0*Q3); d02 = 2.0*(Q1*Q3 + Q0*Q2)
        d10 = 2.0*(Q1*Q2 + Q0*Q3)
        d11 = Q0*Q0 - Q1*Q1 + Q2*Q2 - Q3*Q3
        d12 = 2.0*(Q2*Q3 - Q0*Q1)
        d20 = 2.0*(Q1*Q3 - Q0*Q2); d21 = 2.0*(Q2*Q3 + Q0*Q1)
        d22 = Q0*Q0 - Q1*Q1 - Q2*Q2 + Q3*Q3
        self._dcm00 = d00; self._dcm01 = d01; self._dcm02 = d02
        self._dcm10 = d10; self._dcm11 = d11; self._dcm12 = d12
        self._dcm20 = d20; self._dcm21 = d21; self._dcm22 = d22

        # Write back to numpy arrays for renderer interface
        # レンダラインタフェース用にnumpy配列を書き戻し
        self.uvw[0][0] = self.s_u; self.uvw[1][0] = self.s_v; self.uvw[2][0] = self.s_w
        self.pqr[0][0] = self.s_p; self.pqr[1][0] = self.s_q; self.pqr[2][0] = self.s_r
        self.quat[0][0] = Q0; self.quat[1][0] = Q1
        self.quat[2][0] = Q2; self.quat[3][0] = Q3
        self.position[0][0] = self.s_x; self.position[1][0] = self.s_y; self.position[2][0] = self.s_z

        # Update DCM numpy array
        # DCM numpy配列更新
        self.DCM[0][0] = d00; self.DCM[0][1] = d01; self.DCM[0][2] = d02
        self.DCM[1][0] = d10; self.DCM[1][1] = d11; self.DCM[1][2] = d12
        self.DCM[2][0] = d20; self.DCM[2][1] = d21; self.DCM[2][2] = d22

        # Update euler angles
        # オイラー角更新
        asin_val = -2.0*(Q1*Q3 - Q0*Q2)
        if asin_val > 1.0:
            asin_val = 1.0
        elif asin_val < -1.0:
            asin_val = -1.0
        self.euler[0][0] = math.atan2(2.0*(Q0*Q1 + Q2*Q3), Q0*Q0 - Q1*Q1 - Q2*Q2 + Q3*Q3)
        self.euler[1][0] = math.asin(asin_val)
        self.euler[2][0] = math.atan2(2.0*(Q0*Q3 + Q1*Q2), Q0*Q0 + Q1*Q1 - Q2*Q2 - Q3*Q3)

        # Update velocity (inertial frame)
        # 速度更新（慣性座標系）
        self.velocity[0][0] = d00*self.s_u + d01*self.s_v + d02*self.s_w
        self.velocity[1][0] = d10*self.s_u + d11*self.s_v + d12*self.s_w
        self.velocity[2][0] = d20*self.s_u + d21*self.s_v + d22*self.s_w
