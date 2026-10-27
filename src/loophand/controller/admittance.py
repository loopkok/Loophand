import time
import logging
import numpy as np
import pinocchio as pin
from collections import deque
from typing import Dict, List, Optional
import multiprocessing as mp
from enum import Enum

# ==========================================================
# 0. 引入你的自定义导纳控制器类
# ==========================================================
class AdmittanceController():
    """
    导纳控制器 (加入限幅保护版) - 用户提供
    """
    def __init__(self, _mass, _stiffness, _damping, dt, max_vel=0.5, max_acc=20.0):
        self._mass = np.array(_mass, dtype=float)
        self._stiffness = np.array(_stiffness, dtype=float)
        self._damping = np.array(_damping, dtype=float)
        self.dt = float(dt)
        
        # 安全限幅阈值
        self.max_vel = float(max_vel)
        self.max_acc = float(max_acc)
        
        # 状态变量
        self.x_error = np.zeros(3)   # 虚拟位置偏移
        self.vel_error = np.zeros(3) # 虚拟速度
        self.Force_constant = np.zeros(3)
    
    def update(self, Force_ext: np.ndarray):
        """计算柔顺所需要的位移增量"""
        # 1. 核心导纳公式: a = (F_ext - F_const - D*v - K*x) / M
        force_net = (Force_ext - self.Force_constant - 
                     self._damping * self.vel_error - 
                     self._stiffness * self.x_error)
        
        acc = force_net / (self._mass + 1e-9)
        
        # 2. 加速度限幅
        acc = np.clip(acc, -self.max_acc, self.max_acc)
        
        # 3. 积分得到速度
        self.vel_error += acc * self.dt
        
        # 4. 速度限幅 (关键保护)
        self.vel_error = np.clip(self.vel_error, -self.max_vel, self.max_vel)
        
        # 5. 积分得到位置
        self.x_error += self.vel_error * self.dt
        
        # 返回的是累计的位置偏差，外部也可以直接用 self.vel_error * dt 作为 dx
        return self.x_error.copy()
    
    def set_Force_constant(self, F):
        self.Force_constant = np.array(F, dtype=float)

    def reset(self):
        """重置导纳控制器 (用于接触锚定)"""
        self.vel_error = np.zeros(3)
        self.x_error = np.zeros(3)
