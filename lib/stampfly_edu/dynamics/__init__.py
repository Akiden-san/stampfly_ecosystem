"""
Drone Dynamics Module for Education
教育用ドローンダイナミクスモジュール

Provides symbolic and numerical tools for understanding
drone equations of motion.
"""

from .equations import (
    derive_equations_of_motion,
    hover_condition,
    linearize_at_hover,
    motor_curve,
)

__all__ = [
    "derive_equations_of_motion",
    "hover_condition",
    "linearize_at_hover",
    "motor_curve",
]
