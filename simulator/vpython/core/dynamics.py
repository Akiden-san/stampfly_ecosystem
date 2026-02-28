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
import random
import numpy as np
from . import motors as mp
from . import physics as rb
from . import battery as bt

class multicopter():
    '''
    Rotor configuration:
    4_FL_CW    1_FR_CCW
             X
    3_RL_CCW   2_RR_CW
    '''
    def __init__(self, mass, inersia):
        self.body = rb.rigidbody(mass=mass, inersia=inersia)
        self.mp1 = mp.motor_prop(1)
        self.mp2 = mp.motor_prop(2)
        self.mp3 = mp.motor_prop(3)
        self.mp4 = mp.motor_prop(4)
        self.motor_prop = [self.mp1, self.mp2, self.mp3, self.mp4]
        self.distuerbance_moment = [4.4e-6, 4.4e-6, 4.e-6]
        self.distuerbance_force = [1e-6, 1e-6, 1e-6]
        self.battery = bt.battery()
        # Pre-compute gravity vector (constant, avoids 2000 alloc/s)
        # 重力ベクトルを事前計算（定数なので毎ステップの生成を回避）
        self._gravity = np.array([[0.0], [0.0], [mass * 9.81]])

        # Scalar cache for fast path
        # 高速パス用スカラーキャッシュ
        self._mg = mass * 9.81
        # Motor moment coefficients: moment_L = sum(-loc_y * T), moment_M = sum(loc_x * T)
        # モーターモーメント係数: moment_L = sum(-loc_y * T), moment_M = sum(loc_x * T)
        self._mL1 = -self.mp1._loc_y; self._mL2 = -self.mp2._loc_y
        self._mL3 = -self.mp3._loc_y; self._mL4 = -self.mp4._loc_y
        self._mM1 = self.mp1._loc_x; self._mM2 = self.mp2._loc_x
        self._mM3 = self.mp3._loc_x; self._mM4 = self.mp4._loc_x
        self._mN1 = -self.mp1.rotation_dir; self._mN2 = -self.mp2.rotation_dir
        self._mN3 = -self.mp3.rotation_dir; self._mN4 = -self.mp4.rotation_dir

    def set_disturbance(self, moment, force):
        self.distuerbance_moment = moment
        self.distuerbance_force = force

    def set_pqr(self, pqr):
        self.body.set_pqr(pqr)

    def set_uvw(self, uvw):
        self.body.set_uvw(uvw)

    def set_euler(self, euler):
        euler = np.array(euler)
        self.body.set_euler(euler)
        
    def force_moment(self):
        vel_u = self.body.uvw[0][0]
        vel_v = self.body.uvw[1][0]
        vel_w = self.body.uvw[2][0]
        rate_p = self.body.pqr[0][0]
        rate_q = self.body.pqr[1][0]
        rate_r = self.body.pqr[2][0]
        gravity_body = self.body.DCM.T @ self._gravity
        
        #Moment
        moment = self.mp1.get_moment() + self.mp2.get_moment() + self.mp3.get_moment() + self.mp4.get_moment()
        moment_L = moment[0][0] - 1e-5*np.sign(rate_p)*rate_p**2
        moment_M = moment[1][0] - 1e-5*np.sign(rate_q)*rate_q**2
        moment_N = moment[2][0] - 1e-5*np.sign(rate_r)*rate_r**2 

        #Force
        thrust = self.mp1.get_force() + self.mp2.get_force() + self.mp3.get_force() + self.mp4.get_force()
        fx = thrust[0][0] + gravity_body[0][0] - 0.1e0*np.sign(vel_u)*vel_u**2
        fy = thrust[1][0] + gravity_body[1][0] - 0.1e0*np.sign(vel_v)*vel_v**2
        fz = thrust[2][0] + gravity_body[2][0] - 0.1e0*np.sign(vel_w)*vel_w**2
        
        #Add disturbance
        moment_L += np.random.normal(0, self.distuerbance_moment[0])
        moment_M += np.random.normal(0, self.distuerbance_moment[1])
        moment_N += np.random.normal(0, self.distuerbance_moment[2])
        fx += np.random.normal(0, self.distuerbance_force[0])
        fy += np.random.normal(0, self.distuerbance_force[1])
        fz += np.random.normal(0, self.distuerbance_force[2])

        #Output
        Force = [[fx], [fy], [fz]] 
        Moment = np.array([[moment_L],[moment_M],[moment_N]])
        return Force, Moment

    def step(self,voltage, dt):
        #Body state update
        force, moment = self.force_moment()
        self.body.step(force, moment, dt)
        #Motor and Prop state update
        for index, mp in enumerate(self.motor_prop):
            mp.step(voltage[index], dt)

    def force_moment_fast(self):
        """Compute forces and moments using pure Python scalars.
        純粋なPythonスカラーで力とモーメントを計算"""
        # Motor thrusts and torques (scalar)
        # モーター推力とトルク（スカラー）
        t1 = self.mp1.get_thrust(); t2 = self.mp2.get_thrust()
        t3 = self.mp3.get_thrust(); t4 = self.mp4.get_thrust()
        q1 = self.mp1.get_torque(); q2 = self.mp2.get_torque()
        q3 = self.mp3.get_torque(); q4 = self.mp4.get_torque()

        # Moments from motors using pre-computed coefficients
        # 事前計算済み係数によるモーターモーメント
        moment_L = self._mL1*t1 + self._mL2*t2 + self._mL3*t3 + self._mL4*t4
        moment_M = self._mM1*t1 + self._mM2*t2 + self._mM3*t3 + self._mM4*t4
        moment_N = self._mN1*q1 + self._mN2*q2 + self._mN3*q3 + self._mN4*q4

        # Gravity in body frame: DCM^T @ [0, 0, mg]
        # ボディ座標系の重力: DCM^T @ [0, 0, mg]
        b = self.body
        mg = self._mg
        gx = b._dcm20 * mg
        gy = b._dcm21 * mg
        gz = b._dcm22 * mg

        # Total thrust (all in -z body direction)
        # 全推力（ボディ座標系-z方向）
        total_thrust = -(t1 + t2 + t3 + t4)

        # Force = thrust + gravity + quadratic drag
        # 力 = 推力 + 重力 + 二次抵抗
        su = b.s_u; sv = b.s_v; sw = b.s_w
        fx = gx - 0.1 * math.copysign(su * su, su)
        fy = gy - 0.1 * math.copysign(sv * sv, sv)
        fz = total_thrust + gz - 0.1 * math.copysign(sw * sw, sw)

        # Aerodynamic damping on angular rates
        # 角速度の空力ダンピング
        sp = b.s_p; sq = b.s_q; sr = b.s_r
        moment_L -= 1e-5 * math.copysign(sp * sp, sp)
        moment_M -= 1e-5 * math.copysign(sq * sq, sq)
        moment_N -= 1e-5 * math.copysign(sr * sr, sr)

        # Disturbance (random.gauss instead of np.random.normal)
        # 外乱（np.random.normalの代わりにrandom.gauss）
        dm = self.distuerbance_moment
        df = self.distuerbance_force
        if dm[0] > 0: moment_L += random.gauss(0, dm[0])
        if dm[1] > 0: moment_M += random.gauss(0, dm[1])
        if dm[2] > 0: moment_N += random.gauss(0, dm[2])
        if df[0] > 0: fx += random.gauss(0, df[0])
        if df[1] > 0: fy += random.gauss(0, df[1])
        if df[2] > 0: fz += random.gauss(0, df[2])

        return fx, fy, fz, moment_L, moment_M, moment_N

    def step_fast(self, voltage, dt):
        """Physics step using scalar fast path.
        スカラー高速パスによる物理ステップ"""
        fx, fy, fz, L, M, N = self.force_moment_fast()
        self.body.step_fast(fx, fy, fz, L, M, N, dt)
        for index, mp in enumerate(self.motor_prop):
            mp.step(voltage[index], dt)

    #EOF