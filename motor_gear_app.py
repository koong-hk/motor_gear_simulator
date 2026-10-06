# app.py
import math
import itertools
import json
from pathlib import Path
from typing import List, Dict, Tuple, Optional

import numpy as np
import pandas as pd
import streamlit as st
from itertools import product
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


# ============================================================
# 1. 기본 DB
# ============================================================

MOTOR_DB = [
    {
        "motor_id": "BLDC-A24-30W",
        "motor_type": "BLDC",
        "application": "Small fan, air purifier, small pump",
        "rated_torque_nm": 0.06,    # 정격토크
        "peak_torque_nm": 0.12,     # 피크토크    
        "rated_speed_rpm": 4800,    # 정격속도
        "rated_power_w": 30,        # 정격출력
        "efficiency": 0.82,         # 효율
        "mass_kg": 0.18,            # 질량
        "cost_index": 1.0,          # 비용지수
        "outer_diameter_mm": 36,    # 외경
        "length_mm": 45,            # 길이
        "rotor_inertia_kgm2": 0.000006, 
        "noise_factor": 0.80,
        "precision_factor": 0.70,
    },
    {
        "motor_id": "BLDC-B24-60W",
        "motor_type": "BLDC",
        "application": "Robot auxiliary joint, small actuator",
        "rated_torque_nm": 0.16,
        "peak_torque_nm": 0.32,
        "rated_speed_rpm": 3600,
        "rated_power_w": 60,
        "efficiency": 0.85,
        "mass_kg": 0.35,
        "cost_index": 1.5,
        "outer_diameter_mm": 50,
        "length_mm": 60,
        "rotor_inertia_kgm2": 0.000018,
        "noise_factor": 0.75,
        "precision_factor": 0.80,
    },
    {
        "motor_id": "BLDC-C48-120W",
        "motor_type": "BLDC",
        "application": "Servo drive, mobile robot wheel, compact high-power drive",
        "rated_torque_nm": 0.38,
        "peak_torque_nm": 0.75,
        "rated_speed_rpm": 3000,
        "rated_power_w": 120,
        "efficiency": 0.88,
        "mass_kg": 0.75,
        "cost_index": 2.4,
        "outer_diameter_mm": 63,
        "length_mm": 85,
        "rotor_inertia_kgm2": 0.000055,
        "noise_factor": 0.70,
        "precision_factor": 0.85,
    },
    {
        "motor_id": "STEP-N17-40",
        "motor_type": "Step",
        "application": "Small positioning axis, valve, camera module, 3D printer axis",
        "rated_torque_nm": 0.40,
        "peak_torque_nm": 0.50,
        "rated_speed_rpm": 600,
        "rated_power_w": 25,
        "efficiency": 0.65,
        "mass_kg": 0.28,
        "cost_index": 0.9,
        "outer_diameter_mm": 42,
        "length_mm": 40,
        "rotor_inertia_kgm2": 0.000025,
        "noise_factor": 1.10,
        "precision_factor": 0.90,
    },
    {
        "motor_id": "STEP-N23-80",
        "motor_type": "Step",
        "application": "Low-speed robot joint, linear stage, high-torque positioning",
        "rated_torque_nm": 0.90,
        "peak_torque_nm": 1.10,
        "rated_speed_rpm": 450,
        "rated_power_w": 55,
        "efficiency": 0.62,
        "mass_kg": 0.75,
        "cost_index": 1.4,
        "outer_diameter_mm": 57,
        "length_mm": 76,
        "rotor_inertia_kgm2": 0.000095,
        "noise_factor": 1.20,
        "precision_factor": 0.95,
    },
    {
        "motor_id": "BLDC_TEST_001",
        "motor_type": "BLDC",
        "application": "Small fan, compact air pump, miniature actuator",
        "rated_torque_nm": 0.024,
        "peak_torque_nm": 0.072,
        "rated_speed_rpm": 6000.0,
        "rated_power_w": 15.0,
        "efficiency": 0.78,
        "mass_kg": 0.12,
        "cost_index": 0.7,
        "outer_diameter_mm": 24.0,
        "length_mm": 35.0,
        "rotor_inertia_kgm2": 0.000002,
        "noise_factor": 0.82,
        "precision_factor": 0.65,
    },
    {
        "motor_id": "BLDC_TEST_002",
        "motor_type": "BLDC",
        "application": "Small fan, small pump, compact appliance drive",
        "rated_torque_nm": 0.057,
        "peak_torque_nm": 0.171,
        "rated_speed_rpm": 5000.0,
        "rated_power_w": 30.0,
        "efficiency": 0.82,
        "mass_kg": 0.22,
        "cost_index": 1.0,
        "outer_diameter_mm": 32.0,
        "length_mm": 45.0,
        "rotor_inertia_kgm2": 0.000006,
        "noise_factor": 0.80,
        "precision_factor": 0.70,
    },
    {
        "motor_id": "BLDC_TEST_003",
        "motor_type": "BLDC",
        "application": "Air purifier, small blower, compact actuator",
        "rated_torque_nm": 0.119,
        "peak_torque_nm": 0.357,
        "rated_speed_rpm": 4000.0,
        "rated_power_w": 50.0,
        "efficiency": 0.85,
        "mass_kg": 0.35,
        "cost_index": 1.4,
        "outer_diameter_mm": 42.0,
        "length_mm": 55.0,
        "rotor_inertia_kgm2": 0.000015,
        "noise_factor": 0.76,
        "precision_factor": 0.74,
    },
    {
        "motor_id": "BLDC_TEST_004",
        "motor_type": "BLDC",
        "application": "Robot auxiliary joint, small conveyor, appliance actuator",
        "rated_torque_nm": 0.218,
        "peak_torque_nm": 0.654,
        "rated_speed_rpm": 3500.0,
        "rated_power_w": 80.0,
        "efficiency": 0.86,
        "mass_kg": 0.55,
        "cost_index": 1.9,
        "outer_diameter_mm": 52.0,
        "length_mm": 70.0,
        "rotor_inertia_kgm2": 0.000032,
        "noise_factor": 0.74,
        "precision_factor": 0.78,
    },
    {
        "motor_id": "BLDC_TEST_005",
        "motor_type": "BLDC",
        "application": "Servo drive, mobile robot wheel, medium power actuator",
        "rated_torque_nm": 0.382,
        "peak_torque_nm": 1.146,
        "rated_speed_rpm": 3000.0,
        "rated_power_w": 120.0,
        "efficiency": 0.88,
        "mass_kg": 0.85,
        "cost_index": 2.4,
        "outer_diameter_mm": 60.0,
        "length_mm": 85.0,
        "rotor_inertia_kgm2": 0.000055,
        "noise_factor": 0.70,
        "precision_factor": 0.84,
    },
    {
        "motor_id": "BLDC_TEST_006",
        "motor_type": "BLDC",
        "application": "High power actuator, AGV wheel, robot joint drive",
        "rated_torque_nm": 0.764,
        "peak_torque_nm": 2.292,
        "rated_speed_rpm": 2500.0,
        "rated_power_w": 200.0,
        "efficiency": 0.89,
        "mass_kg": 1.25,
        "cost_index": 3.2,
        "outer_diameter_mm": 75.0,
        "length_mm": 100.0,
        "rotor_inertia_kgm2": 0.000120,
        "noise_factor": 0.68,
        "precision_factor": 0.86,
    },
    {
        "motor_id": "STEP_TEST_001",
        "motor_type": "Step",
        "application": "Small positioning axis, valve actuator, camera module",
        "rated_torque_nm": 0.159,
        "peak_torque_nm": 0.45,
        "rated_speed_rpm": 1200.0,
        "rated_power_w": 20.0,
        "efficiency": 0.65,
        "mass_kg": 0.28,
        "cost_index": 0.8,
        "outer_diameter_mm": 35.0,
        "length_mm": 40.0,
        "rotor_inertia_kgm2": 0.000018,
        "noise_factor": 1.08,
        "precision_factor": 0.86,
    },
    {
        "motor_id": "STEP_TEST_002",
        "motor_type": "Step",
        "application": "Compact linear stage, feeder, low-cost positioning axis",
        "rated_torque_nm": 0.424,
        "peak_torque_nm": 1.20,
        "rated_speed_rpm": 900.0,
        "rated_power_w": 40.0,
        "efficiency": 0.68,
        "mass_kg": 0.45,
        "cost_index": 1.1,
        "outer_diameter_mm": 42.0,
        "length_mm": 48.0,
        "rotor_inertia_kgm2": 0.000035,
        "noise_factor": 1.12,
        "precision_factor": 0.90,
    },
    {
        "motor_id": "STEP_TEST_003",
        "motor_type": "Step",
        "application": "3D printer axis, linear stage, medium torque positioning",
        "rated_torque_nm": 1.273,
        "peak_torque_nm": 3.20,
        "rated_speed_rpm": 600.0,
        "rated_power_w": 80.0,
        "efficiency": 0.70,
        "mass_kg": 0.90,
        "cost_index": 1.7,
        "outer_diameter_mm": 57.0,
        "length_mm": 75.0,
        "rotor_inertia_kgm2": 0.000095,
        "noise_factor": 1.20,
        "precision_factor": 0.95,
    },
    {
        "motor_id": "STEP_TEST_004",
        "motor_type": "Step",
        "application": "High torque positioning, robot joint, heavy linear stage",
        "rated_torque_nm": 2.865,
        "peak_torque_nm": 7.00,
        "rated_speed_rpm": 500.0,
        "rated_power_w": 150.0,
        "efficiency": 0.72,
        "mass_kg": 1.60,
        "cost_index": 2.6,
        "outer_diameter_mm": 86.0,
        "length_mm": 90.0,
        "rotor_inertia_kgm2": 0.000220,
        "noise_factor": 1.28,
        "precision_factor": 0.97,
    },
]


MATERIAL_DB = {
    "Steel": {
        "density_kg_m3": 7850,
        "allowable_bending_mpa": 250,
        "allowable_contact_mpa": 900,
        "elastic_modulus_gpa": 210,
        "poisson_ratio": 0.30,
        "cost_factor": 1.0,
        "cp": 190,
        "noise_factor": 1.0,
    },
    "Aluminum": {
        "density_kg_m3": 2700,
        "allowable_bending_mpa": 100,
        "allowable_contact_mpa": 300,
        "elastic_modulus_gpa": 70,
        "poisson_ratio": 0.33,
        "cost_factor": 1.3,
        "cp": 130,
        "noise_factor": 0.95,
    },
    "Engineering Plastic": {
        "density_kg_m3": 1400,
        "allowable_bending_mpa": 50,
        "allowable_contact_mpa": 120,
        "elastic_modulus_gpa": 3,
        "poisson_ratio": 0.38,
        "cost_factor": 0.8,
        "cp": 70,
        "noise_factor": 0.75,
    },
    "Brass": {
        "density_kg_m3": 8500,
        "allowable_bending_mpa": 120,
        "allowable_contact_mpa": 350,
        "elastic_modulus_gpa": 100,
        "poisson_ratio": 0.34,
        "cost_factor": 1.5,
        "cp": 120,
        "noise_factor": 0.90,
    },
}


# ============================================================
# 2. 유틸 함수
# ============================================================

def step_motor_torque_derating(rpm: float) -> float:
    if rpm <= 300:
        return 1.0
    elif rpm <= 600:
        return 0.8
    elif rpm <= 1000:
        return 0.55
    else:
        return 0.35


def get_available_motor_torque(motor: Dict, operating_rpm: float) -> float:
    if motor["motor_type"] == "Step":
        return motor["rated_torque_nm"] * step_motor_torque_derating(operating_rpm)
    return motor["rated_torque_nm"]


def lewis_y_factor(z: float) -> float:
    y = 0.154 - 0.912 / max(z, 1e-9)
    return max(y, 0.03)


def dynamic_factor_kv(pitch_line_velocity_m_s: float, cv: float = 0.05) -> float:
    return 1.0 + cv * math.sqrt(max(pitch_line_velocity_m_s, 0.0))


def normalize_series(values: np.ndarray) -> np.ndarray:
    vmin = np.min(values)
    vmax = np.max(values)
    return (values - vmin) / (vmax - vmin + 1e-12)


def make_ratio_key_from_stage_pairs(stage_pairs, ndigits=4):
    """
    stage_pairs의 전체 기어비를 동일 기어비 판단용 key로 변환합니다.

    예:
    [(20, 40), (20, 50)] -> 2.0 * 2.5 = 5.0 -> "5.0000"
    """

    total_ratio = 1.0

    for z_drive, z_driven in stage_pairs:
        total_ratio *= float(z_driven) / max(float(z_drive), 1e-12)

    return f"{round(total_ratio, ndigits):.{ndigits}f}"


def make_result_ratio_key(result, ndigits=4):
    """
    result 기준으로 동일 기어 단수 + 동일 기어비 판단용 key를 생성합니다.
    """

    stage_count = int(result.get("stage_count", 0))
    total_stage_ratio = float(result.get("total_stage_ratio", 0.0))

    return (
        stage_count,
        f"{round(total_stage_ratio, ndigits):.{ndigits}f}",
    )


def filter_results_by_same_stage_ratio_limit(
    results,
    max_same_stage_ratio_count=10,
    ratio_ndigits=4,
):
    """
    최종 표시 후보의 다양성 확보용 필터입니다.

    동일한 stage_count에서 동일한 total_stage_ratio를 갖는 결과는
    최대 max_same_stage_ratio_count개까지만 남깁니다.
    """

    filtered_results = []
    ratio_counter = defaultdict(int)

    sorted_results = sorted(
        results,
        key=lambda r: r.get(
            "objective_score",
            r.get(
                "layout_score",
                r.get("layout_area_mm2", 1e18),
            ),
        ),
    )

    for result in sorted_results:
        key = make_result_ratio_key(
            result,
            ndigits=ratio_ndigits,
        )

        if ratio_counter[key] >= max_same_stage_ratio_count:
            continue

        filtered_results.append(result)
        ratio_counter[key] += 1

    return filtered_results


def apply_disabled_objective_items(
    objective_weights,
    disabled_objective_items,
):
    """
    사용자가 비활성화한 최적화 항목의 weight를 0으로 변경합니다.

    disabled_objective_items 예:
    ["mass", "cost", "noise"]
    """

    weights = dict(objective_weights)

    for item in disabled_objective_items:
        if item in weights:
            weights[item] = 0.0

    return weights


def get_active_objective_items(objective_weights: Dict) -> Dict[str, float]:
    """
    weight가 0보다 큰 최적화 항목만 active로 판단합니다.

    반환값:
        {
            "mass": 1.0,
            "cost": 1.5,
            "rpm_error": 3.0,
            ...
        }

    이렇게 dict로 반환해야 evaluate_candidate() 안에서
    active_objectives.get("rpm_error", 0.0) 형태로 사용할 수 있습니다.
    """
    active_items = {}

    if not objective_weights:
        return active_items

    for key, value in objective_weights.items():
        try:
            weight = float(value)
        except Exception:
            continue

        if weight > 0.0:
            active_items[key] = weight

    return active_items


def objective_is_active(objective_weights: Dict, key: str) -> bool:
    """
    특정 objective 항목이 활성화되어 있는지 확인합니다.
    """
    try:
        return float(objective_weights.get(key, 0.0)) > 0.0
    except Exception:
        return False


def has_positive_limit(value) -> bool:
    """
    None, 0 이하 값은 제한 없음으로 판단합니다.
    """
    try:
        return value is not None and float(value) > 0.0
    except Exception:
        return False


def needs_layout_calculation(
    objective_weights: Dict,
    max_box_width_mm: Optional[float],
    max_box_height_mm: Optional[float],
    max_box_area_mm2: Optional[float],
    force_layout_for_display: bool = True,
) -> bool:
    """
    Layout 계산이 필요한지 판단합니다.

    Layout이 필요한 경우:
    - box_area 최적화가 켜져 있음
    - volume 최적화가 켜져 있음
    - 크기 제한이 켜져 있음
    - 상세 화면/시각화를 위해 layout이 필요함
    """
    if force_layout_for_display:
        return True

    if objective_is_active(objective_weights, "box_area"):
        return True

    if objective_is_active(objective_weights, "volume"):
        return True

    if has_positive_limit(max_box_width_mm):
        return True

    if has_positive_limit(max_box_height_mm):
        return True

    if has_positive_limit(max_box_area_mm2):
        return True

    return False


def needs_strength_calculation(
    objective_weights: Dict,
    min_sf_bending: Optional[float],
    min_sf_contact: Optional[float],
) -> bool:
    """
    강도 계산이 필요한지 판단합니다.

    강도 계산이 필요한 경우:
    - safety 최적화가 켜져 있음
    - 굽힘 안전율 제한이 있음
    - 면압 안전율 제한이 있음
    """
    if objective_is_active(objective_weights, "safety"):
        return True

    if has_positive_limit(min_sf_bending):
        return True

    if has_positive_limit(min_sf_contact):
        return True

    return False

# 모터 정격 출력
def torque_to_power_w(torque_nm: float, rpm: float) -> float:
    return torque_nm * 2.0 * math.pi * rpm / 60.0


def safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


# ============================================================
# 3. 기어 계산
# ============================================================

def calc_stage_geometry(
    gear_type: str,
    z_drive: int,
    z_driven: int,
    module_mm: float,
    face_width_mm: float,
    helix_angle_deg: float,
) -> Dict:
    if gear_type == "Helical":
        beta = math.radians(helix_angle_deg)
        mt = module_mm / math.cos(beta)
        zv_drive = z_drive / (math.cos(beta) ** 3)
        zv_driven = z_driven / (math.cos(beta) ** 3)
    else:
        mt = module_mm
        zv_drive = z_drive
        zv_driven = z_driven

    d_drive = mt * z_drive
    d_driven = mt * z_driven
    center_distance = (d_drive + d_driven) / 2.0
    stage_ratio = z_driven / z_drive

    return {
        "module_transverse_mm": mt,
        "zv_drive": zv_drive,
        "zv_driven": zv_driven,
        "d_drive_mm": d_drive,
        "d_driven_mm": d_driven,
        "center_distance_mm": center_distance,
        "stage_ratio": stage_ratio,
        "face_width_mm": face_width_mm,
    }


def calc_stage_strength(
    geometry: Dict,
    input_torque_nm: float,
    input_rpm: float,
    material: Dict,
    module_mm: float,
    face_width_mm: float,
    cv: float,
) -> Dict:
    d_drive_m = geometry["d_drive_mm"] / 1000.0
    d_drive_mm = geometry["d_drive_mm"]

    pitch_velocity = math.pi * d_drive_m * input_rpm / 60.0
    kv = dynamic_factor_kv(pitch_velocity, cv=cv)

    ft = 2.0 * input_torque_nm / max(d_drive_m, 1e-12)
    y = lewis_y_factor(geometry["zv_drive"])

    bending_stress_mpa = ft * kv / max(face_width_mm * module_mm * y, 1e-12)

    cp = material["cp"]
    contact_stress_mpa = cp * math.sqrt(
        max((ft * kv) / max(face_width_mm * d_drive_mm, 1e-12), 0.0)
    )

    sf_bending = material["allowable_bending_mpa"] / max(bending_stress_mpa, 1e-12)
    sf_contact = material["allowable_contact_mpa"] / max(contact_stress_mpa, 1e-12)

    return {
        "pitch_velocity_m_s": pitch_velocity,
        "kv": kv,
        "tangential_force_n": ft,
        "lewis_y": y,
        "bending_stress_mpa": bending_stress_mpa,
        "contact_stress_mpa": contact_stress_mpa,
        "sf_bending": sf_bending,
        "sf_contact": sf_contact,
    }


def estimate_gear_mass_kg(d1_mm, d2_mm, face_width_mm, density_kg_m3):
    r1_m = d1_mm / 2.0 / 1000.0
    r2_m = d2_mm / 2.0 / 1000.0
    b_m = face_width_mm / 1000.0

    vol = math.pi * r1_m**2 * b_m + math.pi * r2_m**2 * b_m

    volume_factor = 0.65
    return vol * density_kg_m3 * volume_factor


# ============================================================
# 4. Box 배치 최적화
# ============================================================
def normalize_angle_deg(theta_deg):
    """
    각도를 -180 ~ 180 범위로 정규화합니다.
    """
    theta = float(theta_deg)

    while theta > 180:
        theta -= 360

    while theta <= -180:
        theta += 360

    return theta


def is_same_angle(theta1_deg, theta2_deg, tolerance_deg=1e-6):
    """
    두 각도가 같은지 판단합니다.
    180도와 -180도처럼 표현만 다른 경우도 고려합니다.
    """
    theta1 = normalize_angle_deg(theta1_deg)
    theta2 = normalize_angle_deg(theta2_deg)

    diff = abs(theta1 - theta2)
    diff = min(diff, 360 - diff)

    return diff <= tolerance_deg


def get_angle_relation_keys(stage_angles_deg, tolerance_deg=1e-6):
    """
    stage 간 각도 관계를 key로 변환합니다.

    예:
    [0, 0, 90]      -> ["S1_S2_same", "S2_S3_diff"]
    [45, 45, 45]    -> ["S1_S2_same", "S2_S3_same"]
    [0, 90, 180]    -> ["S1_S2_diff", "S2_S3_diff"]
    """

    keys = []

    for i in range(len(stage_angles_deg) - 1):
        theta_a = stage_angles_deg[i]
        theta_b = stage_angles_deg[i + 1]

        if is_same_angle(theta_a, theta_b, tolerance_deg=tolerance_deg):
            relation = "same"
        else:
            relation = "diff"

        key = f"S{i + 1}_S{i + 2}_{relation}"
        keys.append(key)

    return keys

def can_add_angle_candidate(
    stage_angles_deg,
    relation_counter,
    max_same_relation_count=20,
    tolerance_deg=1e-6,
):
    """
    현재 각도 조합 후보를 candidate_results에 추가해도 되는지 판단합니다.

    max_same_relation_count:
        특정 stage pair의 same 조건을 최대 몇 개까지 허용할지 결정합니다.
    """

    relation_keys = get_angle_relation_keys(
        stage_angles_deg,
        tolerance_deg=tolerance_deg,
    )

    for key in relation_keys:
        if key.endswith("_same"):
            if relation_counter[key] >= max_same_relation_count:
                return False

    return True


def update_angle_relation_counter(
    stage_angles_deg,
    relation_counter,
    tolerance_deg=1e-6,
):
    """
    후보가 실제로 추가된 뒤 stage 간 각도 관계 counter를 업데이트합니다.
    """

    relation_keys = get_angle_relation_keys(
        stage_angles_deg,
        tolerance_deg=tolerance_deg,
    )

    for key in relation_keys:
        relation_counter[key] += 1


def calc_center_distance_mm(drive_pitch_diameter_mm, driven_pitch_diameter_mm):
    """
    두 기어의 중심 거리 계산.
    """
    return 0.5 * (drive_pitch_diameter_mm + driven_pitch_diameter_mm)


def polar_to_xy(distance_mm, theta_deg):
    """
    거리와 각도를 x, y 변위로 변환합니다.

    theta_deg = 0   : +X 방향
    theta_deg = 90  : +Y 방향
    theta_deg = 180 : -X 방향
    theta_deg = -90 : -Y 방향
    """
    theta_rad = math.radians(theta_deg)

    dx = distance_mm * math.cos(theta_rad)
    dy = distance_mm * math.sin(theta_rad)

    return dx, dy


def build_layout_with_angles(
    motor,
    stage_details,
    stage_angles_deg,
    motor_position=(0.0, 0.0),
):
    if len(stage_details) != len(stage_angles_deg):
        raise ValueError("stage_details와 stage_angles_deg의 길이가 같아야 합니다.")

    motor_x, motor_y = motor_position

    motor_length = float(motor.get("length_mm", 50.0))
    motor_diameter = float(motor.get("outer_diameter_mm", 40.0))

    circles = []

    circles.append(
        {
            "type": "motor",
            "name": motor.get("motor_id", "Motor"),
            "x": motor_x,
            "y": motor_y,
            "r": motor_diameter / 2.0,
            "length_mm": motor_length,
            "outer_diameter_mm": motor_diameter,
        }
    )

    current_x = motor_x
    current_y = motor_y

    for i, stage in enumerate(stage_details):
        stage_no = int(stage.get("stage", i + 1))
        theta_deg = float(stage_angles_deg[i])

        drive_pitch_diameter = float(stage.get("d_drive_mm", 0.0))
        driven_pitch_diameter = float(stage.get("d_driven_mm", 0.0))

        drive_outer_diameter = float(
            stage.get("drive_outer_diameter_mm", drive_pitch_diameter)
        )
        driven_outer_diameter = float(
            stage.get("driven_outer_diameter_mm", driven_pitch_diameter)
        )

        drive_radius = drive_outer_diameter / 2.0
        driven_radius = driven_outer_diameter / 2.0

        center_distance = float(
            stage.get(
                "center_distance_mm",
                calc_center_distance_mm(
                    drive_pitch_diameter,
                    driven_pitch_diameter,
                ),
            )
        )

        dx, dy = polar_to_xy(center_distance, theta_deg)

        drive_x = current_x
        drive_y = current_y

        driven_x = drive_x + dx
        driven_y = drive_y + dy

        circles.append(
            {
                "type": "gear_drive",
                "name": f"S{stage_no} Drive",
                "stage": stage_no,
                "x": drive_x,
                "y": drive_y,
                "r": drive_radius,
                "theta_deg": theta_deg,
                "center_distance_mm": center_distance,
                "dx_to_driven_mm": dx,
                "dy_to_driven_mm": dy,
            }
        )

        circles.append(
            {
                "type": "gear_driven",
                "name": f"S{stage_no} Driven",
                "stage": stage_no,
                "x": driven_x,
                "y": driven_y,
                "r": driven_radius,
                "theta_deg": theta_deg,
                "center_distance_mm": center_distance,
                "dx_from_drive_mm": dx,
                "dy_from_drive_mm": dy,
            }
        )

        current_x = driven_x
        current_y = driven_y

    output_x = current_x
    output_y = current_y

    motor_to_output_dx = output_x - motor_x
    motor_to_output_dy = output_y - motor_y
    motor_to_output_distance = math.hypot(motor_to_output_dx, motor_to_output_dy)

    bbox = calc_layout_bbox(circles)

    layout = {
        "circles": circles,
        "bbox": bbox,
        "stage_angles_deg": list(stage_angles_deg),
        "angles": list(stage_angles_deg),
        "motor_axis": {
            "x": motor_x,
            "y": motor_y,
        },
        "output_axis": {
            "x": output_x,
            "y": output_y,
        },
        "motor_to_output": {
            "dx_mm": motor_to_output_dx,
            "dy_mm": motor_to_output_dy,
            "distance_mm": motor_to_output_distance,
        },
    }

    return layout


def calc_layout_bbox(circles):
    """
    모터와 기어를 포함하는 bounding box를 계산합니다.
    """

    x_mins = []
    x_maxs = []
    y_mins = []
    y_maxs = []

    for c in circles:
        ctype = c.get("type", "")
        x = float(c.get("x", 0.0))
        y = float(c.get("y", 0.0))

        if ctype == "motor":
            length_mm = float(c.get("length_mm", c.get("r", 1.0) * 2.0))
            diameter_mm = float(c.get("outer_diameter_mm", c.get("r", 1.0) * 2.0))

            x_mins.append(x - length_mm / 2.0)
            x_maxs.append(x + length_mm / 2.0)
            y_mins.append(y - diameter_mm / 2.0)
            y_maxs.append(y + diameter_mm / 2.0)
        else:
            r = float(c.get("r", 1.0))

            x_mins.append(x - r)
            x_maxs.append(x + r)
            y_mins.append(y - r)
            y_maxs.append(y + r)

    min_x = min(x_mins)
    max_x = max(x_maxs)
    min_y = min(y_mins)
    max_y = max(y_maxs)

    width = max_x - min_x
    height = max_y - min_y
    area = width * height

    return {
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
        "width": width,
        "height": height,
        "area": area,
    }


def calc_layout_score(result, weight_distance=0.0):
    """
    layout bounding box 면적과 모터-출력축 거리를 이용해 layout score를 계산합니다.

    score = area + weight_distance * motor_to_output_distance
    """

    layout = result.get("layout", {})
    bbox = layout.get("bbox", {})
    motor_to_output = layout.get("motor_to_output", {})

    area = float(bbox.get("area", 1e18))
    motor_to_output_distance = float(
        motor_to_output.get("distance_mm", 0.0)
    )

    score = area + weight_distance * motor_to_output_distance

    return score


def optimize_layout_angles_with_diversity_limit(
    motor,
    stage_details,
    angle_candidates_deg=None,
    max_same_relation_count=20,
    top_k=50,
    tolerance_deg=1e-6,
):
    """
    stage 각도 조합을 탐색하되,
    stage 간 동일 각도 조건이 너무 많이 반복되지 않도록 제한합니다.

    예:
    S1-S2 동일 각도 후보가 20개를 넘으면,
    이후 S1-S2 동일 각도 후보는 제외하고 다른 각도 조합을 탐색합니다.
    """

    if angle_candidates_deg is None:
        angle_candidates_deg = [-135, -90, -45, 0, 45, 90, 135, 180]

    num_stages = len(stage_details)

    layout_candidates = []
    relation_counter = defaultdict(int)

    for stage_angles_tuple in product(angle_candidates_deg, repeat=num_stages):
        stage_angles_deg = list(stage_angles_tuple)

        if not can_add_angle_candidate(
            stage_angles_deg=stage_angles_deg,
            relation_counter=relation_counter,
            max_same_relation_count=max_same_relation_count,
            tolerance_deg=tolerance_deg,
        ):
            continue

        layout = build_layout_with_angles(
            motor=motor,
            stage_details=stage_details,
            stage_angles_deg=stage_angles_deg,
            motor_position=(0.0, 0.0),
        )

        bbox = layout.get("bbox", {})
        layout_area_mm2 = float(bbox.get("area", 1e18))

        layout_candidate = {
            "layout": layout,
            "best_angles_deg": stage_angles_deg,
            "layout_area_mm2": layout_area_mm2,
            "angle_relation_keys": get_angle_relation_keys(
                stage_angles_deg,
                tolerance_deg=tolerance_deg,
            ),
        }

        layout_candidates.append(layout_candidate)

        update_angle_relation_counter(
            stage_angles_deg=stage_angles_deg,
            relation_counter=relation_counter,
            tolerance_deg=tolerance_deg,
        )

    layout_candidates = sorted(
        layout_candidates,
        key=lambda x: x["layout_area_mm2"],
    )

    return layout_candidates[:top_k]

def get_stage_distance_table(result):
    """
    result에서 stage별 거리 및 각도 정보를 추출합니다.
    """

    layout = result.get("layout", {})
    circles = layout.get("circles", [])

    rows = []

    driven_by_stage = {
        c.get("stage"): c
        for c in circles
        if c.get("type") == "gear_driven"
    }

    drive_by_stage = {
        c.get("stage"): c
        for c in circles
        if c.get("type") == "gear_drive"
    }

    stages = sorted(set(drive_by_stage.keys()) & set(driven_by_stage.keys()))

    for stage in stages:
        drive = drive_by_stage[stage]
        driven = driven_by_stage[stage]

        dx = float(driven.get("x", 0.0)) - float(drive.get("x", 0.0))
        dy = float(driven.get("y", 0.0)) - float(drive.get("y", 0.0))
        distance = math.hypot(dx, dy)

        rows.append(
            {
                "stage": stage,
                "drive_gear": drive.get("name", ""),
                "driven_gear": driven.get("name", ""),
                "theta_deg": drive.get("theta_deg", ""),
                "dx_mm": dx,
                "dy_mm": dy,
                "center_distance_mm": distance,
            }
        )

    return rows


# ============================================================
# 5. 후보 평가
# ============================================================
# 최적화 평가 함수
def evaluate_candidate(
    motor: Dict,
    gear_type: str,
    material_name: str,
    stage_pairs: List[Tuple[int, int]],
    module_mm: float,
    face_width_mm: float,
    stage_efficiency: float,
    external_gearbox_ratio: float,
    external_gearbox_efficiency: float,
    helix_angle_deg: float,
    cv: float,
    min_sf_bending: Optional[float],
    min_sf_contact: Optional[float],
    target_output_rpm: float,
    target_output_torque: Optional[float],
    ratio_min: float,
    ratio_max: float,
    rpm_tolerance: float,
    max_center_distance_mm: float,
    max_face_width_mm: float,
    max_box_width_mm: Optional[float],
    max_box_height_mm: Optional[float],
    max_box_area_mm2: Optional[float],
    objective_weights: Dict,
    application_profile: str,
    output_torque_min: Optional[float] = None,
    output_torque_max: Optional[float] = None,
    force_layout_for_display: bool = True,
) -> List[Dict]:

    material = MATERIAL_DB[material_name]
    active_objectives = get_active_objective_items(objective_weights)

    need_strength = needs_strength_calculation(
        objective_weights=objective_weights,
        min_sf_bending=min_sf_bending,
        min_sf_contact=min_sf_contact,
    )

    need_layout = needs_layout_calculation(
        objective_weights=objective_weights,
        max_box_width_mm=max_box_width_mm,
        max_box_height_mm=max_box_height_mm,
        max_box_area_mm2=max_box_area_mm2,
        force_layout_for_display=force_layout_for_display,
    )

    need_mass = (
        "mass" in active_objectives
        or "cost" in active_objectives
        or "volume" in active_objectives
    )

    need_cost = "cost" in active_objectives
    need_loss = "loss" in active_objectives
    need_noise = (
        "noise" in active_objectives
        or application_profile == "가전제품"
    )
    need_precision = (
        application_profile == "로봇/서보"
        and (
            "precision" in active_objectives
            or "backlash" in active_objectives
        )
    )
    need_backlash = (
        application_profile == "로봇/서보"
        and "backlash" in active_objectives
    )

    total_stage_ratio = 1.0
    stages = []

    rpm_in = motor["rated_speed_rpm"]
    torque_in = get_available_motor_torque(motor, rpm_in)

    min_bending_sf = 1e9
    min_contact_sf = 1e9
    max_bending_stress = 0.0
    max_contact_stress = 0.0
    total_gear_mass = 0.0
    total_gear_cost = 0.0

    stage_details = []

    if face_width_mm > max_face_width_mm:
        return []

    for idx, (z_drive, z_driven) in enumerate(stage_pairs, start=1):
        geom = calc_stage_geometry(
            gear_type=gear_type,
            z_drive=z_drive,
            z_driven=z_driven,
            module_mm=module_mm,
            face_width_mm=face_width_mm,
            helix_angle_deg=helix_angle_deg,
        )

        if geom["center_distance_mm"] > max_center_distance_mm:
            return []

        stage_ratio = geom["stage_ratio"]
        rpm_out = rpm_in / stage_ratio
        torque_out = torque_in * stage_ratio * stage_efficiency

        strength = None

        if need_strength or need_noise:
            strength = calc_stage_strength(
                geometry=geom,
                input_torque_nm=torque_in,
                input_rpm=rpm_in,
                material=material,
                module_mm=module_mm,
                face_width_mm=face_width_mm,
                cv=cv,
            )

            if min_sf_bending is not None and float(min_sf_bending) > 0:
                if strength["sf_bending"] < float(min_sf_bending):
                    return []

            if min_sf_contact is not None and float(min_sf_contact) > 0:
                if strength["sf_contact"] < float(min_sf_contact):
                    return []

            min_bending_sf = min(min_bending_sf, strength["sf_bending"])
            min_contact_sf = min(min_contact_sf, strength["sf_contact"])
            max_bending_stress = max(max_bending_stress, strength["bending_stress_mpa"])
            max_contact_stress = max(max_contact_stress, strength["contact_stress_mpa"])

        gear_mass = 0.0

        if need_mass:
            gear_mass = estimate_gear_mass_kg(
                geom["d_drive_mm"],
                geom["d_driven_mm"],
                face_width_mm,
                material["density_kg_m3"],
            )
            total_gear_mass += gear_mass

            if need_cost:
                total_gear_cost += gear_mass * material["cost_factor"] * 10.0

        total_stage_ratio *= stage_ratio

        stage_record = {
            "stage": idx,
            "z_drive": z_drive,
            "z_driven": z_driven,
            "stage_ratio": stage_ratio,
            "module_mm": module_mm,
            "face_width_mm": face_width_mm,
            "d_drive_mm": geom["d_drive_mm"],
            "d_driven_mm": geom["d_driven_mm"],
            "center_distance_mm": geom["center_distance_mm"],
            "rpm_in": rpm_in,
            "rpm_out": rpm_out,
            "torque_in_nm": torque_in,
            "torque_out_nm": torque_out,
            "pitch_velocity_m_s": strength["pitch_velocity_m_s"] if strength else 0.0,
            "kv": strength["kv"] if strength else 0.0,
            "tangential_force_n": strength["tangential_force_n"] if strength else 0.0,
            "lewis_y": strength["lewis_y"] if strength else 0.0,
            "bending_stress_mpa": strength["bending_stress_mpa"] if strength else 0.0,
            "contact_stress_mpa": strength["contact_stress_mpa"] if strength else 0.0,
            "sf_bending": strength["sf_bending"] if strength else None,
            "sf_contact": strength["sf_contact"] if strength else None,
            "gear_mass_kg": gear_mass,
        }

        stage_details.append(stage_record)

        stage_geom_for_layout = {
            "d_drive_mm": geom["d_drive_mm"],
            "d_driven_mm": geom["d_driven_mm"],
            "center_distance_mm": geom["center_distance_mm"],
        }
        stages.append(stage_geom_for_layout)

        rpm_in = rpm_out
        torque_in = torque_out

    total_ratio = external_gearbox_ratio * total_stage_ratio

    if total_ratio < ratio_min or total_ratio > ratio_max:
        return []

    output_rpm = motor["rated_speed_rpm"] / total_ratio

    rpm_error = abs(output_rpm - target_output_rpm) / max(target_output_rpm, 1e-9)

    if rpm_error > rpm_tolerance:
        return []

    gear_total_efficiency = external_gearbox_efficiency * (
        stage_efficiency ** len(stage_pairs)
    )
    total_efficiency = motor["efficiency"] * gear_total_efficiency

    output_torque = (
        get_available_motor_torque(motor, motor["rated_speed_rpm"])
        * total_ratio
        * gear_total_efficiency
    )

    if target_output_torque is not None and target_output_torque > 0:
        if output_torque < target_output_torque:
            return []

    if output_torque_min is not None and output_torque < output_torque_min:
        return []

    if output_torque_max is not None and output_torque > output_torque_max:
        return []

    input_power_w = 0.0
    output_power_w = 0.0
    loss_w = 0.0

    if need_loss:
        input_power_w = torque_to_power_w(
            get_available_motor_torque(motor, motor["rated_speed_rpm"]),
            motor["rated_speed_rpm"],
        )

        output_power_w = torque_to_power_w(output_torque, output_rpm)
        loss_w = max(input_power_w - output_power_w, 0.0)

    if need_layout:
        layout_candidates = optimize_layout_angles_with_diversity_limit(
            motor=motor,
            stage_details=stage_details,
            angle_candidates_deg=[-135, -90, -45, 0, 45, 90, 135, 180],
            max_same_relation_count=20,
            top_k=50,
        )
    else:
        layout_candidates = [
            {
                "layout": None,
                "best_angles_deg": [],
                "layout_area_mm2": 0.0,
                "angle_relation_keys": [],
            }
        ]

    if not layout_candidates:
        return []

    results = []

    for layout_candidate in layout_candidates:
        layout = layout_candidate["layout"]
        best_angles_deg = layout_candidate["best_angles_deg"]
        layout_area_mm2 = layout_candidate["layout_area_mm2"]
        angle_relation_keys = layout_candidate["angle_relation_keys"]

        motor_to_output_distance_mm = 0.0
        layout_score = 0.0
        box_width = 0.0
        box_height = 0.0
        box_area = 0.0

        if layout is not None:
            motor_to_output_distance_mm = layout.get(
                "motor_to_output",
                {},
            ).get(
                "distance_mm",
                0.0,
            )

            layout_score = layout_area_mm2 + 10.0 * motor_to_output_distance_mm

            box_width = layout["bbox"]["width"]
            box_height = layout["bbox"]["height"]
            box_area = layout["bbox"]["area"]

            if max_box_width_mm is not None and max_box_width_mm > 0:
                if box_width > max_box_width_mm:
                    continue

            if max_box_height_mm is not None and max_box_height_mm > 0:
                if box_height > max_box_height_mm:
                    continue

            if max_box_area_mm2 is not None and max_box_area_mm2 > 0:
                if box_area > max_box_area_mm2:
                    continue

        total_mass = 0.0
        total_cost = 0.0
        volume_proxy = 0.0

        if "mass" in active_objectives or need_mass:
            total_mass = motor["mass_kg"] + total_gear_mass

        if "cost" in active_objectives:
            total_cost = motor["cost_index"] + total_gear_cost

        if "volume" in active_objectives:
            volume_proxy = box_area * face_width_mm if layout is not None else 0.0

        gear_noise_factor = 0.0
        backlash_factor = 0.0
        noise_score = 0.0
        backlash_score = 0.0
        stiffness_score = 0.0
        precision_score = 0.0

        if gear_type == "Helical":
            gear_noise_factor = 0.75
            backlash_factor = 0.85
        else:
            gear_noise_factor = 1.0
            backlash_factor = 1.0

        if need_noise:
            material_noise_factor = material["noise_factor"]

            avg_pitch_velocity = (
                np.mean([s["pitch_velocity_m_s"] for s in stage_details])
                if stage_details
                else 0.0
            )

            noise_score = (
                motor["noise_factor"]
                * gear_noise_factor
                * material_noise_factor
                * (1.0 + 0.08 * avg_pitch_velocity)
                * (1.0 + 0.05 * len(stage_pairs))
            )

        if need_precision or need_backlash:
            backlash_score = backlash_factor * len(stage_pairs) / max(module_mm, 1e-9)

            stiffness_score = face_width_mm * (module_mm ** 2) / max(
                np.mean([s["center_distance_mm"] for s in stage_details]),
                1e-9,
            )

            precision_score = (
                motor["precision_factor"]
                + stiffness_score
                - 0.1 * backlash_score
            )

        safety_penalty = 0.0
        over_design_penalty = 0.0

        if "safety" in active_objectives and need_strength:
            safety_penalty += max(
                0.0,
                float(min_sf_bending or 0.0) - min_bending_sf,
            ) ** 2
            safety_penalty += max(
                0.0,
                float(min_sf_contact or 0.0) - min_contact_sf,
            ) ** 2

            over_design_penalty += max(0.0, min_bending_sf - 8.0) * 0.05
            over_design_penalty += max(0.0, min_contact_sf - 8.0) * 0.05

        # ------------------------------------------------------------
        # Penalty / score 계산
        # ------------------------------------------------------------

        rpm_error_pct = rpm_error * 100.0

        torque_shortage_penalty = 0.0
        torque_excess_penalty = 0.0

        if target_output_torque is not None and target_output_torque > 0:
            torque_shortage_penalty = max(
                0.0,
                (target_output_torque - output_torque) / max(target_output_torque, 1e-9),
            ) ** 2

            torque_excess_penalty = max(
                0.0,
                (output_torque - target_output_torque) / max(target_output_torque, 1e-9),
            ) * 0.02

        efficiency_penalty = max(0.0, 1.0 - total_efficiency)

        mass_score = total_mass
        cost_score = total_cost
        volume_score = volume_proxy
        loss_score = loss_w

        ratio_error_score = abs(total_ratio - ((motor["rated_speed_rpm"] / max(target_output_rpm, 1e-9)))) / max(
            (motor["rated_speed_rpm"] / max(target_output_rpm, 1e-9)),
            1e-9,
        )

        # layout_score는 layout이 없으면 0.0입니다.
        # 단, layout 목적함수가 활성화되어 있는데 layout이 없는 경우에는 불리하게 처리합니다.
        if "layout" in active_objectives and layout is None:
            layout_score = 1e9

        # volume 목적함수가 활성화되어 있는데 layout이 없는 경우에도 불리하게 처리합니다.
        if "volume" in active_objectives and layout is None:
            volume_score = 1e9

        # ------------------------------------------------------------
        # Objective score 계산
        # ------------------------------------------------------------

        objective_score = 0.0

        if "rpm" in active_objectives:
            objective_score += float(active_objectives.get("rpm", 0.0)) * rpm_error_pct

        if "rpm_error" in active_objectives:
            objective_score += float(active_objectives.get("rpm_error", 0.0)) * rpm_error_pct

        if "ratio" in active_objectives:
            objective_score += float(active_objectives.get("ratio", 0.0)) * ratio_error_score

        if "torque" in active_objectives:
            objective_score += (
                float(active_objectives.get("torque", 0.0))
                * (torque_shortage_penalty + torque_excess_penalty)
            )

        if "torque_margin" in active_objectives:
            objective_score += (
                float(active_objectives.get("torque_margin", 0.0))
                * torque_excess_penalty
            )

        if "efficiency" in active_objectives:
            objective_score += (
                float(active_objectives.get("efficiency", 0.0))
                * efficiency_penalty
            )

        if "loss" in active_objectives:
            objective_score += float(active_objectives.get("loss", 0.0)) * loss_score

        if "mass" in active_objectives:
            objective_score += float(active_objectives.get("mass", 0.0)) * mass_score

        if "cost" in active_objectives:
            objective_score += float(active_objectives.get("cost", 0.0)) * cost_score

        if "volume" in active_objectives:
            objective_score += float(active_objectives.get("volume", 0.0)) * volume_score

        if "layout" in active_objectives:
            objective_score += float(active_objectives.get("layout", 0.0)) * layout_score

        if "box_area" in active_objectives:
            objective_score += float(active_objectives.get("box_area", 0.0)) * box_area

        if "noise" in active_objectives:
            objective_score += float(active_objectives.get("noise", 0.0)) * noise_score

        if "backlash" in active_objectives:
            objective_score += float(active_objectives.get("backlash", 0.0)) * backlash_score

        if "precision" in active_objectives:
            # precision_score는 클수록 좋은 값이므로 penalty 형태로 변환합니다.
            precision_penalty = 1.0 / max(precision_score, 1e-9)
            objective_score += (
                float(active_objectives.get("precision", 0.0))
                * precision_penalty
            )

        if "safety" in active_objectives:
            objective_score += (
                float(active_objectives.get("safety", 0.0))
                * safety_penalty
            )

        if "over_design" in active_objectives:
            objective_score += (
                float(active_objectives.get("over_design", 0.0))
                * over_design_penalty
            )

        # ------------------------------------------------------------
        # 결과 저장
        # ------------------------------------------------------------

        result = {
            "motor": motor,
            "motor_id": motor.get("motor_id"),
            "motor_name": motor.get("motor_name", motor.get("motor_id")),
            "motor_type": motor.get("motor_type"),
            "gear_type": gear_type,
            "material": material_name,
            "application_profile": application_profile,

            "stage_count": len(stage_pairs),
            "stage_pairs": stage_pairs,
            "stage_details": stage_details,

            "module_mm": module_mm,
            "face_width_mm": face_width_mm,
            "helix_angle_deg": helix_angle_deg,

            "total_stage_ratio": total_stage_ratio,
            "external_gearbox_ratio": external_gearbox_ratio,
            "total_ratio": total_ratio,

            "motor_rpm": motor["rated_speed_rpm"],
            "output_rpm": output_rpm,
            "target_output_rpm": target_output_rpm,
            "rpm_error": rpm_error,
            "rpm_error_pct": rpm_error_pct,

            "output_torque": output_torque,
            "output_torque_nm": output_torque,
            "target_output_torque": target_output_torque,
            "target_output_torque_nm": target_output_torque,

            "stage_efficiency": stage_efficiency,
            "external_gearbox_efficiency": external_gearbox_efficiency,
            "gear_total_efficiency": gear_total_efficiency,
            "total_efficiency": total_efficiency,

            "input_power_w": input_power_w,
            "output_power_w": output_power_w,
            "loss_w": loss_w,

            "min_bending_sf": None if min_bending_sf >= 1e9 else min_bending_sf,
            "min_contact_sf": None if min_contact_sf >= 1e9 else min_contact_sf,
            "max_bending_stress_mpa": max_bending_stress,
            "max_contact_stress_mpa": max_contact_stress,

            "total_gear_mass_kg": total_gear_mass,
            "motor_mass_kg": motor.get("mass_kg", 0.0),
            "total_mass_kg": total_mass,

            "total_gear_cost": total_gear_cost,
            "motor_cost_index": motor.get("cost_index", 0.0),
            "total_cost": total_cost,
            "total_cost_index": total_cost,

            "layout": layout,
            "best_angles_deg": best_angles_deg,
            "angle_relation_keys": angle_relation_keys,
            "layout_area_mm2": layout_area_mm2,
            "layout_score": layout_score,
            "motor_to_output_distance_mm": motor_to_output_distance_mm,

            "box_width_mm": box_width,
            "box_height_mm": box_height,
            "box_area_mm2": box_area,
            "volume_proxy": volume_proxy,

            "noise_score": noise_score,
            "backlash_score": backlash_score,
            "stiffness_score": stiffness_score,
            "precision_score": precision_score,

            "safety_penalty": safety_penalty,
            "over_design_penalty": over_design_penalty,
            "torque_shortage_penalty": torque_shortage_penalty,
            "torque_excess_penalty": torque_excess_penalty,
            "efficiency_penalty": efficiency_penalty,
            "ratio_error_score": ratio_error_score,

            "objective_score": objective_score,
            "score": objective_score,
        }

        results.append(result)

    return results




def get_angle_signature(stage_angles_deg):
    """
    각도 조합 자체를 signature로 만듭니다.

    예:
    [0, 45, -90] -> "0_45_-90"
    """

    return "_".join(str(int(round(a))) for a in stage_angles_deg)

def select_top_results_with_angle_diversity(
    candidate_results,
    top_k=20,
    max_same_angle_signature_count=2,
    max_same_relation_count=20,
    weight_distance=0.0,
):
    """
    최종 후보 중 layout score가 좋은 순서로 정렬하되,
    동일 각도 조합 또는 동일 stage 관계가 상위 결과에 과도하게 반복되지 않도록 제한합니다.
    """

    scored_results = []

    for result in candidate_results:
        score = calc_layout_score(
            result,
            weight_distance=weight_distance,
        )

        result["layout_score"] = score
        scored_results.append(result)

    scored_results = sorted(
        scored_results,
        key=lambda x: x["layout_score"],
    )

    selected_results = []
    angle_signature_counter = defaultdict(int)
    relation_counter = defaultdict(int)

    for result in scored_results:
        stage_angles_deg = result.get("best_angles_deg", [])

        if not stage_angles_deg:
            layout = result.get("layout", {})
            stage_angles_deg = layout.get("stage_angles_deg", [])

        angle_signature = get_angle_signature(stage_angles_deg)

        if angle_signature_counter[angle_signature] >= max_same_angle_signature_count:
            continue

        relation_keys = get_angle_relation_keys(stage_angles_deg)

        skip = False

        for key in relation_keys:
            if key.endswith("_same"):
                if relation_counter[key] >= max_same_relation_count:
                    skip = True
                    break

        if skip:
            continue

        selected_results.append(result)
        angle_signature_counter[angle_signature] += 1

        for key in relation_keys:
            relation_counter[key] += 1

        if len(selected_results) >= top_k:
            break

    return selected_results

# ============================================================
# 6. Grid Search
# ============================================================

def generate_stage_pairs(
    z_min: int,
    z_max: int,
    min_pinion_teeth: int,
    max_stage_ratio: float,
    pair_limit: int = 300,
) -> List[Tuple[int, int]]:
    pairs = []

    for z_drive in range(max(z_min, min_pinion_teeth), z_max + 1):
        for z_driven in range(z_drive + 1, z_max + 1):
            ratio = z_driven / z_drive

            if ratio <= 1.0:
                continue

            if ratio > max_stage_ratio:
                continue

            pairs.append((z_drive, z_driven))

    pairs = sorted(pairs, key=lambda x: x[1] / x[0])

    if len(pairs) > pair_limit:
        idx = np.linspace(0, len(pairs) - 1, pair_limit).astype(int)
        pairs = [pairs[i] for i in idx]

    return pairs


def run_grid_search(
    motors: List[Dict],
    gear_type: str,
    material_name: str,
    stage_count: int,
    module_candidates: List[float],
    face_width_candidates: List[float],
    z_min: int,
    z_max: int,
    min_pinion_teeth: int,
    max_stage_ratio: float,
    stage_efficiency: float,
    external_gearbox_ratio: float,
    external_gearbox_efficiency: float,
    helix_angle_deg: float,
    cv: float,
    min_sf_bending: float,
    min_sf_contact: float,
    target_output_rpm: float,
    target_output_torque: Optional[float],
    ratio_min: float,
    ratio_max: float,
    rpm_tolerance: float,
    max_center_distance_mm: float,
    max_face_width_mm: float,
    max_box_width_mm: Optional[float],
    max_box_height_mm: Optional[float],
    max_box_area_mm2: Optional[float],
    objective_weights: Dict,
    application_profile: str,
    use_motor_rpm_filter: bool = False,
    output_torque_min: Optional[float] = None,
    output_torque_max: Optional[float] = None,
    motor_rpm_min: float = 0.0,
    motor_rpm_max: float = 100000.0,
    max_candidates: int = 50000,
    max_same_ratio_eval_count: int = 100,
    max_same_ratio_result_count: int = 10,
    ratio_key_ndigits: int = 4,
) -> List[Dict]:
    base_pairs = generate_stage_pairs(
        z_min=z_min,
        z_max=z_max,
        min_pinion_teeth=min_pinion_teeth,
        max_stage_ratio=max_stage_ratio,
    )

    if not base_pairs:
        return []

    filtered_motors = []

    for motor in motors:
        if use_motor_rpm_filter:
            rpm = float(motor.get("rated_speed_rpm", 0.0))
            if rpm < motor_rpm_min or rpm > motor_rpm_max:
                continue

        filtered_motors.append(motor)

    if not filtered_motors:
        return []

    results = []
    checked = 0

    # 동일 기어 단수에서 동일 총 기어비 조합을 너무 많이 평가하지 않기 위한 counter입니다.
    # key: (stage_count, total_stage_ratio_key)
    same_ratio_eval_counter = defaultdict(int)
  

    eval_kwargs = {
        "gear_type": gear_type,
        "material_name": material_name,
        "stage_efficiency": stage_efficiency,
        "external_gearbox_ratio": external_gearbox_ratio,
        "external_gearbox_efficiency": external_gearbox_efficiency,
        "helix_angle_deg": helix_angle_deg,
        "cv": cv,
        "min_sf_bending": min_sf_bending,
        "min_sf_contact": min_sf_contact,
        "target_output_rpm": target_output_rpm,
        "target_output_torque": target_output_torque,
        "ratio_min": ratio_min,
        "ratio_max": ratio_max,
        "rpm_tolerance": rpm_tolerance,
        "max_center_distance_mm": max_center_distance_mm,
        "max_face_width_mm": max_face_width_mm,
        "max_box_width_mm": max_box_width_mm,
        "max_box_height_mm": max_box_height_mm,
        "max_box_area_mm2": max_box_area_mm2,
        "objective_weights": objective_weights,
        "application_profile": application_profile,
    }

    pair_combinations = itertools.product(base_pairs, repeat=stage_count)

    for stage_pairs in pair_combinations:
        stage_pairs = tuple(stage_pairs)

        ratio_key = make_ratio_key_from_stage_pairs(
            stage_pairs,
            ndigits=ratio_key_ndigits,
        )

        same_ratio_key = (
            int(stage_count),
            ratio_key,
        )

        # 동일 기어 단수 + 동일 총 기어비 조합이 100개를 넘으면
        # 더 이상 이 동일 기어비 조합은 평가하지 않고 다른 기어비 조합으로 넘어갑니다.
        if same_ratio_eval_counter[same_ratio_key] >= max_same_ratio_eval_count:
            continue

        checked += 1
        same_ratio_eval_counter[same_ratio_key] += 1

        if checked > max_candidates:
            break

        for motor in filtered_motors:
            for module_mm in module_candidates:
                for face_width_mm in face_width_candidates:
                    candidates = evaluate_candidate(
                        motor=motor,
                        stage_pairs=list(stage_pairs),
                        module_mm=module_mm,
                        face_width_mm=face_width_mm,
                        output_torque_min=output_torque_min,
                        output_torque_max=output_torque_max,
                        **eval_kwargs,
                    )

                    if candidates:
                        if isinstance(candidates, list):
                            results.extend(candidates)
                        else:
                            results.append(candidates)
    if not results:
        return []

    results = sorted(
        results,
        key=lambda r: r.get(
            "objective_score",
            r.get(
                "layout_score",
                r.get("layout_area_mm2", 1e18),
            ),
        ),
    )

    # 최종 결과 다양성 확보:
    # 동일 stage_count + 동일 total_stage_ratio 결과는 10개까지만 유지합니다.
    results = filter_results_by_same_stage_ratio_limit(
        results=results,
        max_same_stage_ratio_count=max_same_ratio_result_count,
        ratio_ndigits=ratio_key_ndigits,
    )

    results = select_top_results_with_angle_diversity(
        candidate_results=results,
        top_k=max_candidates,
        max_same_angle_signature_count=2,
        max_same_relation_count=20,
        weight_distance=0.0,
    )

    return results



# ============================================================
# 7. 시각화 함수
# ============================================================

def plot_stage_torque_rpm(stage_details: List[Dict]):
    df = pd.DataFrame(stage_details)

    fig, ax1 = plt.subplots(figsize=(4, 2.5))

    ax1.plot(df["stage"], df["torque_out_nm"], marker="o", label="Output Torque [Nm]")
    ax1.set_xlabel("Stage")
    ax1.set_ylabel("Torque [Nm]")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(df["stage"], df["rpm_out"], marker="s", color="tab:red", label="Output RPM")
    ax2.set_ylabel("RPM")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="best")

    fig.tight_layout()
    return fig


def plot_stage_stress_sf(stage_details: List[Dict]):
    df = pd.DataFrame(stage_details)

    fig, ax1 = plt.subplots(figsize=(8, 4))

    ax1.bar(df["stage"] - 0.15, df["bending_stress_mpa"], width=0.3, label="Bending Stress [MPa]")
    ax1.bar(df["stage"] + 0.15, df["contact_stress_mpa"], width=0.3, label="Contact Stress [MPa]")
    ax1.set_xlabel("Stage")
    ax1.set_ylabel("Stress [MPa]")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(df["stage"], df["sf_bending"], marker="o", color="tab:green", label="Bending SF")
    ax2.plot(df["stage"], df["sf_contact"], marker="s", color="tab:purple", label="Contact SF")
    ax2.set_ylabel("Safety Factor")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="best")

    fig.tight_layout()
    return fig


def plot_pcd_circles(stage_details: List[Dict]):
    fig, ax = plt.subplots(figsize=(8, 4))

    x = 0.0

    for _, s in enumerate(stage_details):
        r1 = s["d_drive_mm"] / 2.0
        r2 = s["d_driven_mm"] / 2.0

        c1 = Circle((x + r1, r1), r1, fill=False, linewidth=2)
        ax.add_patch(c1)
        ax.text(x + r1, r1, f"S{s['stage']} Drive\nD={s['d_drive_mm']:.1f}", ha="center", va="center", fontsize=8)

        x += 2 * r1 + 10

        c2 = Circle((x + r2, r2), r2, fill=False, linewidth=2, linestyle="--")
        ax.add_patch(c2)
        ax.text(x + r2, r2, f"S{s['stage']} Driven\nD={s['d_driven_mm']:.1f}", ha="center", va="center", fontsize=8)

        x += 2 * r2 + 20

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, max(x, 100))
    ax.set_ylim(0, max([s["d_driven_mm"] for s in stage_details] + [100]))
    ax.set_title("PCD Circle Comparison")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_layout(result):
    layout = result.get("layout", {})
    circles = layout.get("circles", [])
    bbox = layout.get("bbox", {})

    if not circles or not bbox:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.text(
            0.5,
            0.5,
            "No layout data available",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )
        ax.axis("off")
        return fig

    box_x_min = bbox["min_x"]
    box_x_max = bbox["max_x"]
    box_y_min = bbox["min_y"]
    box_y_max = bbox["max_y"]
    box_width = bbox["width"]
    box_height = bbox["height"]

    fig_width = max(4.0, min(7.0, box_width / 50.0))
    fig_height = max(2.5, min(4.0, box_height / 40.0))

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    box_rect = Rectangle(
        (box_x_min, box_y_min),
        box_width,
        box_height,
        fill=False,
        linestyle="--",
        linewidth=2,
        edgecolor="black",
    )
    ax.add_patch(box_rect)

    motor = result.get("motor", {})
    motor_id = motor.get("motor_id", "Motor")
    motor_length = float(motor.get("length_mm", 50.0))
    motor_diameter = float(motor.get("outer_diameter_mm", 40.0))

    for c in circles:
        ctype = c.get("type", "")
        name = c.get("name", "")
        x = float(c.get("x", 0.0))
        y = float(c.get("y", 0.0))
        r = float(c.get("r", 1.0))

        if ctype == "motor":
            motor_rect = Rectangle(
                (x - motor_length / 2.0, y - motor_diameter / 2.0),
                motor_length,
                motor_diameter,
                fill=True,
                alpha=0.25,
                facecolor="tab:blue",
                edgecolor="tab:blue",
                linewidth=1.5,
            )
            ax.add_patch(motor_rect)
            ax.text(
                x,
                y,
                f"Motor\n{motor_id}",
                ha="center",
                va="center",
                fontsize=8,
            )
        else:
            color = "tab:orange" if ctype == "gear_drive" else "tab:green"
            gear_circle = Circle(
                (x, y),
                r,
                fill=True,
                alpha=0.30,
                facecolor=color,
                edgecolor=color,
                linewidth=1.5,
            )
            ax.add_patch(gear_circle)
            ax.plot(x, y, marker="o", color="black", markersize=3)
            ax.text(
                x,
                y,
                name,
                ha="center",
                va="center",
                fontsize=7,
            )

    gear_circles = [c for c in circles if c.get("type") != "motor"]

    drive_by_stage = {
        c.get("stage"): c
        for c in gear_circles
        if c.get("type") == "gear_drive"
    }

    driven_by_stage = {
        c.get("stage"): c
        for c in gear_circles
        if c.get("type") == "gear_driven"
    }

    stages = sorted(set(drive_by_stage.keys()) & set(driven_by_stage.keys()))

    for stage in stages:
        c1 = drive_by_stage[stage]
        c2 = driven_by_stage[stage]

        x1 = float(c1.get("x", 0.0))
        y1 = float(c1.get("y", 0.0))
        x2 = float(c2.get("x", 0.0))
        y2 = float(c2.get("y", 0.0))

        ax.plot(
            [x1, x2],
            [y1, y2],
            color="gray",
            linestyle="-",
            linewidth=1.0,
            alpha=0.7,
        )

        mid_x = 0.5 * (x1 + x2)
        mid_y = 0.5 * (y1 + y2)

        dx = x2 - x1
        dy = y2 - y1
        distance = math.hypot(dx, dy)
        theta_deg = math.degrees(math.atan2(dy, dx))

        ax.text(
            mid_x,
            mid_y,
            f"S{stage}\n{theta_deg:.0f}°\n{distance:.1f} mm",
            ha="center",
            va="center",
            fontsize=7,
            color="dimgray",
            bbox={
                "boxstyle": "round,pad=0.2",
                "facecolor": "white",
                "edgecolor": "none",
                "alpha": 0.7,
            },
        )

    output_axis = layout.get("output_axis", {})
    output_x = float(output_axis.get("x", 0.0))
    output_y = float(output_axis.get("y", 0.0))

    ax.plot([output_x], [output_y], marker="x", color="tab:red", markersize=8)
    ax.text(
        output_x,
        output_y,
        " Output Shaft",
        ha="left",
        va="bottom",
        fontsize=8,
        color="tab:red",
    )

    motor_axis = layout.get("motor_axis", {})
    motor_axis_x = float(motor_axis.get("x", 0.0))
    motor_axis_y = float(motor_axis.get("y", 0.0))

    ax.plot([motor_axis_x], [motor_axis_y], marker="+", color="tab:blue", markersize=8)
    ax.text(
        motor_axis_x,
        motor_axis_y,
        " Motor Axis",
        ha="left",
        va="top",
        fontsize=8,
        color="tab:blue",
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_title(
        f"Optimized Layout Box: {box_width:.1f} mm × {box_height:.1f} mm"
    )
    ax.grid(True, alpha=0.25)

    margin = 5.0
    ax.set_xlim(box_x_min - margin, box_x_max + margin)
    ax.set_ylim(box_y_min - margin, box_y_max + margin)

    ax.legend(
        handles=[
            Rectangle(
                (0, 0),
                1,
                1,
                fill=True,
                alpha=0.25,
                facecolor="tab:blue",
                edgecolor="tab:blue",
                label="Motor",
            ),
            Circle(
                (0, 0),
                1,
                fill=True,
                alpha=0.30,
                facecolor="tab:orange",
                edgecolor="tab:orange",
                label="Drive Gear",
            ),
            Circle(
                (0, 0),
                1,
                fill=True,
                alpha=0.30,
                facecolor="tab:green",
                edgecolor="tab:green",
                label="Driven Gear",
            ),
        ],
        loc="best",
        fontsize=8,
    )

    fig.tight_layout()

    return fig


def show_half_width_pyplot(fig, center: bool = True):
    """
    Streamlit pyplot figure를 현재 화면 폭의 약 50% 수준으로 표시합니다.

    center=True  : 가운데 정렬, 전체 폭의 약 50%
    center=False : 왼쪽 정렬, 전체 폭의 약 50%
    """

    if center:
        col_left, col_fig, col_right = st.columns([1, 2, 1])

        with col_fig:
            st.pyplot(
                fig,
                use_container_width=True,
            )
    else:
        col_fig, col_empty = st.columns([1, 1])

        with col_fig:
            st.pyplot(
                fig,
                use_container_width=True,
            )

# ============================================================
# 8. 단순화 UI용 Preset / 추천 함수
# ============================================================

PRESET = {
    "가전제품": {
        "gear_type": "Helical",
        "material": "Engineering Plastic",
        "stage_efficiency": 0.92,
        "external_gearbox_ratio": 1.0,
        "external_gearbox_efficiency": 1.0,
        "helix_angle_deg": 15.0,
        "cv": 0.05,
        "min_sf_bending": 1.3,
        "min_sf_contact": 1.2,
        "rpm_tolerance": 0.05,
        "z_min": 17,
        "z_max": 60,
        "min_pinion_teeth": 17,
        "max_stage_ratio": 5.0,
        "ratio_min": 1.0,
        "ratio_max": 300.0,
        "max_center_distance_mm": 80.0,
        "max_candidates": 50000,
        "objective_weights": {
            "mass": 1.0,
            "cost": 1.5,
            "volume": 1.5,
            "loss": 1.0,
            "noise": 3.0,
            "safety": 1.0,
            "box_area": 2.0,
            "rpm_error": 3.0,
            "precision": 0.0,
            "backlash": 0.0,
        },
    },
    "로봇/서보": {
        "gear_type": "Helical",
        "material": "Steel",
        "stage_efficiency": 0.94,
        "external_gearbox_ratio": 1.0,
        "external_gearbox_efficiency": 1.0,
        "helix_angle_deg": 15.0,
        "cv": 0.05,
        "min_sf_bending": 1.8,
        "min_sf_contact": 1.5,
        "rpm_tolerance": 0.03,
        "z_min": 17,
        "z_max": 70,
        "min_pinion_teeth": 17,
        "max_stage_ratio": 5.0,
        "ratio_min": 1.0,
        "ratio_max": 500.0,
        "max_center_distance_mm": 100.0,
        "max_candidates": 80000,
        "objective_weights": {
            "mass": 1.0,
            "cost": 0.7,
            "volume": 1.0,
            "loss": 1.0,
            "noise": 1.0,
            "safety": 2.0,
            "box_area": 1.0,
            "rpm_error": 4.0,
            "precision": 2.0,
            "backlash": 2.0,
        },
    },
}


def recommend_stage_count(required_ratio: float) -> int:
    if required_ratio <= 5:
        return 1
    elif required_ratio <= 25:
        return 2
    elif required_ratio <= 100:
        return 3
    elif required_ratio <= 300:
        return 4
    else:
        return 5


def recommend_stage_count_for_motors(
    motors: List[Dict],
    target_output_rpm: float,
) -> int:
    if not motors:
        return 2

    required_ratios = [
        m["rated_speed_rpm"] / max(target_output_rpm, 1e-9)
        for m in motors
    ]

    median_ratio = float(np.median(required_ratios))
    return recommend_stage_count(median_ratio)


def recommend_module_range(target_torque_nm: float) -> List[float]:
    if target_torque_nm <= 0.5:
        return [0.4, 0.5, 0.6, 0.8]
    elif target_torque_nm <= 2.0:
        return [0.6, 0.8, 1.0, 1.25]
    elif target_torque_nm <= 5.0:
        return [1.0, 1.25, 1.5, 2.0]
    elif target_torque_nm <= 10.0:
        return [1.5, 2.0, 2.5, 3.0]
    else:
        return [2.0, 2.5, 3.0, 4.0]


def recommend_face_width_range(module_candidates: List[float]) -> List[float]:
    values = []

    for m in module_candidates:
        for k in [6, 8, 10, 12]:
            values.append(round(m * k, 1))

    return sorted(list(set(values)))


def recommend_ratio_range_for_motors(
    motors: List[Dict],
    target_output_rpm: float,
    margin: float = 0.4,
) -> Tuple[float, float]:
    if not motors:
        return 1.0, 300.0

    required_ratios = [
        m["rated_speed_rpm"] / max(target_output_rpm, 1e-9)
        for m in motors
    ]

    ratio_min = max(1.0, min(required_ratios) * (1.0 - margin))
    ratio_max = max(required_ratios) * (1.0 + margin)

    return ratio_min, ratio_max


def build_module_candidates(
    module_min: float,
    module_max: float,
    module_step: float,
) -> List[float]:
    return np.arange(
        module_min,
        module_max + 1e-9,
        module_step,
    ).round(4).tolist()


def build_face_width_candidates(
    face_width_min: float,
    face_width_max: float,
    face_width_step: float,
) -> List[float]:
    return np.arange(
        face_width_min,
        face_width_max + 1e-9,
        face_width_step,
    ).round(4).tolist()

# ============================================================
# 8. Streamlit UI
# ============================================================

st.set_page_config(
    page_title="Motor-Gear Optimization Simulator",
    layout="wide",
)

# ============================================================
# Session State 초기화
# ============================================================
if "results" not in st.session_state:
    st.session_state.results = []

if "summary_df" not in st.session_state:
    st.session_state.summary_df = pd.DataFrame()

if "top_results" not in st.session_state:
    st.session_state.top_results = []

if "last_result_count" not in st.session_state:
    st.session_state.last_result_count = 10

st.title("모터-기어 조합 최적화 시뮬레이터")
st.caption("입력 항목을 간단모드 / 표준모드 / 고급모드로 구분하여 모터와 기어 조합을 최적화합니다.")

with st.sidebar:
    st.header("1. 입력 모드")

    input_mode = st.radio(
        "입력 모드 선택",
        ["간단모드", "표준모드", "고급모드"],
        index=0,
    )

    application_profile = st.selectbox(
        "적용 분야",
        ["가전제품", "로봇/서보"],
        index=0,
    )

    preset = PRESET[application_profile]

    st.sidebar.subheader("Motor Filtering")

    use_motor_rpm_filter = st.sidebar.checkbox(
        "모터 정격 RPM 범위 제한",
        value=True,
    )

    motor_rpm_min = 0.0
    motor_rpm_max = 100000.0

    if use_motor_rpm_filter:
        motor_rpm_min, motor_rpm_max = st.sidebar.slider(
            "Motor Rated Speed Range [rpm]",
            min_value=0.0,
            max_value=20000.0,
            value=(0.0, 10000.0),
            step=100.0,
        )


    st.sidebar.subheader("Output Filtering")

    use_output_torque_filter = st.sidebar.checkbox(
        "출력 토크 범위 제한",
        value=True,
    )

    output_torque_min = 0.0
    output_torque_max = 100000.0

    if use_output_torque_filter:
        output_torque_min, output_torque_max = st.sidebar.slider(
            "Output Torque Range [Nm]",
            min_value=0.0,
            max_value=500.0,
            value=(0.0, 50.0),
            step=0.5,
        )

    st.header("2. 목표 출력 조건")

    target_output_rpm = st.number_input(
        "목표 출력 RPM",
        min_value=1.0,
        max_value=10000.0,
        value=300.0,
        step=10.0,
    )

    target_output_torque = st.number_input(
        "목표 출력 Torque [Nm]",
        min_value=0.0,
        max_value=1000.0,
        value=1.0,
        step=0.1,
    )

    rpm_tolerance = preset["rpm_tolerance"]

    if input_mode in ["표준모드", "고급모드"]:
        rpm_tolerance_pct = st.slider(
            "Output RPM 허용 오차 [%]",
            min_value=1.0,
            max_value=30.0,
            value=float(preset["rpm_tolerance"] * 100.0),
            step=1.0,
        )
        rpm_tolerance = rpm_tolerance_pct / 100.0

    st.header("3. 모터 선택")

    motor_df = pd.DataFrame(MOTOR_DB)

    motor_select_mode = st.radio(
        "모터 선택 방식",
        ["자동 선택", "직접 선택"],
        index=0,
    )

    if motor_select_mode == "직접 선택":
        motor_id = st.selectbox(
            "모터 선택",
            motor_df["motor_id"].tolist(),
        )

        selected_motors = [
            m for m in MOTOR_DB
            if m["motor_id"] == motor_id
        ]
    else:
        if input_mode == "고급모드":
            motor_type_filter = st.multiselect(
                "모터 타입 필터",
                ["BLDC", "Step"],
                default=["BLDC", "Step"],
            )

            selected_motors = [
                m for m in MOTOR_DB
                if m["motor_type"] in motor_type_filter
            ]
        else:
            selected_motors = MOTOR_DB

    with st.expander("선택된 모터 DB 보기", expanded=False):
        st.dataframe(
            pd.DataFrame(selected_motors),
            use_container_width=True,
        )

    st.header("4. 크기 제한")

    use_size_limit = st.checkbox(
        "크기 제한 사용",
        value=False,
    )

    if use_size_limit:
        max_box_width_mm = st.number_input(
            "최대 Box Width [mm]",
            min_value=1.0,
            max_value=1000.0,
            value=120.0,
            step=10.0,
        )

        max_box_height_mm = st.number_input(
            "최대 Box Height [mm]",
            min_value=1.0,
            max_value=1000.0,
            value=120.0,
            step=10.0,
        )
    else:
        max_box_width_mm = None
        max_box_height_mm = None

    max_box_area_mm2 = None

    if input_mode == "고급모드":
        max_box_area_input = st.number_input(
            "최대 Box Area [mm²], 0이면 미사용",
            min_value=0.0,
            max_value=1000000.0,
            value=0.0,
            step=1000.0,
        )

        max_box_area_mm2 = max_box_area_input if max_box_area_input > 0 else None

    st.header("5. 기어 기본 설정")

    if input_mode == "간단모드":
        gear_type_option = st.selectbox(
            "Gear Type",
            ["자동", "Spur", "Helical"],
            index=0,
        )

        if gear_type_option == "자동":
            gear_type = preset["gear_type"]
        else:
            gear_type = gear_type_option

        material_name = preset["material"]
        stage_efficiency = preset["stage_efficiency"]
        external_gearbox_ratio = preset["external_gearbox_ratio"]
        external_gearbox_efficiency = preset["external_gearbox_efficiency"]
        helix_angle_deg = preset["helix_angle_deg"] if gear_type == "Helical" else 0.0
        cv = preset["cv"]
        min_sf_bending = preset["min_sf_bending"]
        min_sf_contact = preset["min_sf_contact"]

        module_candidates = recommend_module_range(target_output_torque)
        face_width_candidates = recommend_face_width_range(module_candidates)

        recommended_stage_count = recommend_stage_count_for_motors(
            selected_motors,
            target_output_rpm,
        )

        # 간단모드 = 2단, 3단, 4단 전체 탐색
        # 2단에서 결과가 나와도 중단하지 않고 3단, 4단까지 모두 탐색한 뒤 최종 최적 결과를 표시
        stage_count_candidates = [2, 3, 4]

        z_min = preset["z_min"]
        z_max = preset["z_max"]
        min_pinion_teeth = preset["min_pinion_teeth"]
        max_stage_ratio = preset["max_stage_ratio"]

        ratio_min, ratio_max = recommend_ratio_range_for_motors(
            selected_motors,
            target_output_rpm,
            margin=0.5,
        )

        max_center_distance_mm = preset["max_center_distance_mm"]
        max_face_width_mm = max(face_width_candidates)

        objective_weights = preset["objective_weights"]
        max_candidates = preset["max_candidates"]

        st.info(
            f"자동 설정: {gear_type}, {material_name}, "
            f"추천 Stage {recommended_stage_count}단, "
            f"실제 탐색 Stage {stage_count_candidates}, "
            f"Module 후보 {module_candidates}, "
            f"Face Width 후보 {face_width_candidates}"
        )

    elif input_mode == "표준모드":
        gear_type_option = st.selectbox(
            "Gear Type",
            ["자동", "Spur", "Helical"],
            index=0,
        )

        if gear_type_option == "자동":
            gear_type = preset["gear_type"]
        else:
            gear_type = gear_type_option

        material_name = st.selectbox(
            "Gear Material",
            list(MATERIAL_DB.keys()),
            index=list(MATERIAL_DB.keys()).index(preset["material"]),
        )

        auto_stage_count = recommend_stage_count_for_motors(
            selected_motors,
            target_output_rpm,
        )

        stage_count = st.slider(
            "Stage Count",
            min_value=1,
            max_value=5,
            value=int(auto_stage_count),
            step=1,
        )

        stage_count_candidates = [int(stage_count)]

        module_candidates_auto = recommend_module_range(target_output_torque)
        default_module_min = min(module_candidates_auto)
        default_module_max = max(module_candidates_auto)

        module_min = st.number_input(
            "Module Min [mm]",
            min_value=0.1,
            max_value=10.0,
            value=float(default_module_min),
            step=0.1,
        )

        module_max = st.number_input(
            "Module Max [mm]",
            min_value=0.1,
            max_value=10.0,
            value=float(default_module_max),
            step=0.1,
        )

        module_step = st.number_input(
            "Module Step [mm]",
            min_value=0.1,
            max_value=5.0,
            value=0.2,
            step=0.1,
        )

        module_candidates = build_module_candidates(
            module_min,
            module_max,
            module_step,
        )

        face_width_candidates_auto = recommend_face_width_range(module_candidates_auto)
        default_face_width_min = min(face_width_candidates_auto)
        default_face_width_max = max(face_width_candidates_auto)

        face_width_min = st.number_input(
            "Face Width Min [mm]",
            min_value=1.0,
            max_value=100.0,
            value=float(default_face_width_min),
            step=1.0,
        )

        face_width_max = st.number_input(
            "Face Width Max [mm]",
            min_value=1.0,
            max_value=100.0,
            value=float(default_face_width_max),
            step=1.0,
        )

        face_width_step = st.number_input(
            "Face Width Step [mm]",
            min_value=1.0,
            max_value=20.0,
            value=2.0,
            step=1.0,
        )

        face_width_candidates = build_face_width_candidates(
            face_width_min,
            face_width_max,
            face_width_step,
        )

        min_sf_bending = st.number_input(
            "Minimum Bending Safety Factor",
            min_value=0.1,
            max_value=20.0,
            value=float(preset["min_sf_bending"]),
            step=0.1,
        )

        min_sf_contact = st.number_input(
            "Minimum Contact Safety Factor",
            min_value=0.1,
            max_value=20.0,
            value=float(preset["min_sf_contact"]),
            step=0.1,
        )

        z_min = st.number_input(
            "Minimum Tooth Count",
            min_value=8,
            max_value=200,
            value=int(preset["z_min"]),
            step=1,
        )

        z_max = st.number_input(
            "Maximum Tooth Count",
            min_value=10,
            max_value=300,
            value=int(preset["z_max"]),
            step=1,
        )

        min_pinion_teeth = st.number_input(
            "Minimum Pinion Teeth",
            min_value=8,
            max_value=100,
            value=int(preset["min_pinion_teeth"]),
            step=1,
        )

        max_stage_ratio = st.number_input(
            "Maximum Stage Ratio",
            min_value=1.1,
            max_value=20.0,
            value=float(preset["max_stage_ratio"]),
            step=0.1,
        )

        ratio_min_auto, ratio_max_auto = recommend_ratio_range_for_motors(
            selected_motors,
            target_output_rpm,
            margin=0.5,
        )

        ratio_min = st.number_input(
            "Minimum Total Ratio",
            min_value=1.0,
            max_value=1000.0,
            value=float(round(ratio_min_auto, 2)),
            step=0.5,
        )

        ratio_max = st.number_input(
            "Maximum Total Ratio",
            min_value=1.0,
            max_value=2000.0,
            value=float(round(ratio_max_auto, 2)),
            step=0.5,
        )

        max_center_distance_mm = st.number_input(
            "Maximum Center Distance per Stage [mm]",
            min_value=1.0,
            max_value=500.0,
            value=float(preset["max_center_distance_mm"]),
            step=5.0,
        )

        max_face_width_mm = max(face_width_candidates)

        stage_efficiency = preset["stage_efficiency"]
        external_gearbox_ratio = 1.0
        external_gearbox_efficiency = 1.0
        helix_angle_deg = preset["helix_angle_deg"] if gear_type == "Helical" else 0.0
        cv = preset["cv"]
        objective_weights = preset["objective_weights"]
        max_candidates = preset["max_candidates"]

    else:
        gear_type = st.selectbox(
            "Gear Type",
            ["Spur", "Helical"],
            index=1 if preset["gear_type"] == "Helical" else 0,
        )

        material_name = st.selectbox(
            "Gear Material",
            list(MATERIAL_DB.keys()),
            index=list(MATERIAL_DB.keys()).index(preset["material"]),
        )

        stage_count = st.slider(
            "Stage Count",
            min_value=1,
            max_value=5,
            value=2,
            step=1,
        )

        # 고급모드는 사용자가 선택한 단수 하나만 탐색
        stage_count_candidates = [int(stage_count)]

        module_min = st.number_input(
            "Module Min [mm]",
            min_value=0.1,
            max_value=10.0,
            value=0.5,
            step=0.1,
        )

        module_max = st.number_input(
            "Module Max [mm]",
            min_value=0.1,
            max_value=10.0,
            value=1.5,
            step=0.1,
        )

        module_step = st.number_input(
            "Module Step [mm]",
            min_value=0.1,
            max_value=5.0,
            value=0.5,
            step=0.1,
        )

        module_candidates = build_module_candidates(
            module_min,
            module_max,
            module_step,
        )

        face_width_min = st.number_input(
            "Face Width Min [mm]",
            min_value=1.0,
            max_value=100.0,
            value=4.0,
            step=1.0,
        )

        face_width_max = st.number_input(
            "Face Width Max [mm]",
            min_value=1.0,
            max_value=100.0,
            value=10.0,
            step=1.0,
        )

        face_width_step = st.number_input(
            "Face Width Step [mm]",
            min_value=1.0,
            max_value=20.0,
            value=2.0,
            step=1.0,
        )

        face_width_candidates = build_face_width_candidates(
            face_width_min,
            face_width_max,
            face_width_step,
        )

        if gear_type == "Helical":
            helix_angle_deg = st.slider(
                "Helix Angle [deg]",
                min_value=10.0,
                max_value=30.0,
                value=float(preset["helix_angle_deg"]),
                step=1.0,
            )
        else:
            helix_angle_deg = 0.0

        stage_efficiency = st.slider(
            "Stage Efficiency",
            min_value=0.70,
            max_value=0.99,
            value=float(preset["stage_efficiency"]),
            step=0.01,
        )

        use_external_gearbox = st.checkbox(
            "External Gearbox 사용",
            value=False,
        )

        if use_external_gearbox:
            external_gearbox_ratio = st.number_input(
                "External Gearbox Ratio",
                min_value=1.0,
                max_value=500.0,
                value=1.0,
                step=0.5,
            )

            external_gearbox_efficiency = st.slider(
                "External Gearbox Efficiency",
                min_value=0.50,
                max_value=1.00,
                value=0.90,
                step=0.01,
            )
        else:
            external_gearbox_ratio = 1.0
            external_gearbox_efficiency = 1.0

        z_min = st.number_input(
            "Minimum Tooth Count",
            min_value=8,
            max_value=200,
            value=int(preset["z_min"]),
            step=1,
        )

        z_max = st.number_input(
            "Maximum Tooth Count",
            min_value=10,
            max_value=300,
            value=int(preset["z_max"]),
            step=1,
        )

        min_pinion_teeth = st.number_input(
            "Minimum Pinion Teeth",
            min_value=8,
            max_value=100,
            value=int(preset["min_pinion_teeth"]),
            step=1,
        )

        max_stage_ratio = st.number_input(
            "Maximum Stage Ratio",
            min_value=1.1,
            max_value=20.0,
            value=float(preset["max_stage_ratio"]),
            step=0.1,
        )

        ratio_min_auto, ratio_max_auto = recommend_ratio_range_for_motors(
            selected_motors,
            target_output_rpm,
            margin=0.5,
        )

        ratio_min = st.number_input(
            "Minimum Total Ratio",
            min_value=1.0,
            max_value=1000.0,
            value=float(round(ratio_min_auto, 2)),
            step=0.5,
        )

        ratio_max = st.number_input(
            "Maximum Total Ratio",
            min_value=1.0,
            max_value=2000.0,
            value=float(round(ratio_max_auto, 2)),
            step=0.5,
        )

        min_sf_bending = st.number_input(
            "Minimum Bending Safety Factor",
            min_value=0.1,
            max_value=20.0,
            value=float(preset["min_sf_bending"]),
            step=0.1,
        )

        min_sf_contact = st.number_input(
            "Minimum Contact Safety Factor",
            min_value=0.1,
            max_value=20.0,
            value=float(preset["min_sf_contact"]),
            step=0.1,
        )

        max_center_distance_mm = st.number_input(
            "Maximum Center Distance per Stage [mm]",
            min_value=1.0,
            max_value=500.0,
            value=float(preset["max_center_distance_mm"]),
            step=5.0,
        )

        max_face_width_mm = st.number_input(
            "Maximum Face Width per Stage [mm]",
            min_value=1.0,
            max_value=200.0,
            value=max(face_width_candidates) if face_width_candidates else 10.0,
            step=1.0,
        )

        cv = st.number_input(
            "Dynamic Factor Cv",
            min_value=0.0,
            max_value=1.0,
            value=float(preset["cv"]),
            step=0.01,
        )

        max_candidates = st.number_input(
            "Max Candidate Combination",
            min_value=100,
            max_value=500000,
            value=int(preset["max_candidates"]),
            step=1000,
        )

        st.subheader("목적함수 Weight")

        weight_mass = st.slider(
            "Mass Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["mass"]),
            0.1,
        )

        weight_cost = st.slider(
            "Cost Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["cost"]),
            0.1,
        )

        weight_volume = st.slider(
            "Volume Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["volume"]),
            0.1,
        )

        weight_loss = st.slider(
            "Loss Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["loss"]),
            0.1,
        )

        weight_noise = st.slider(
            "Noise Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["noise"]),
            0.1,
        )

        weight_safety = st.slider(
            "Safety Penalty Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["safety"]),
            0.1,
        )

        weight_box_area = st.slider(
            "Box Area Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["box_area"]),
            0.1,
        )

        weight_rpm_error = st.slider(
            "RPM Error Weight",
            0.0,
            10.0,
            float(preset["objective_weights"]["rpm_error"]),
            0.1,
        )

        if application_profile == "로봇/서보":
            weight_precision = st.slider(
                "Precision Weight",
                0.0,
                10.0,
                float(preset["objective_weights"]["precision"]),
                0.1,
            )

            weight_backlash = st.slider(
                "Backlash Weight",
                0.0,
                10.0,
                float(preset["objective_weights"]["backlash"]),
                0.1,
            )
        else:
            weight_precision = 0.0
            weight_backlash = 0.0

        objective_weights = {
            "mass": weight_mass,
            "cost": weight_cost,
            "volume": weight_volume,
            "loss": weight_loss,
            "noise": weight_noise,
            "safety": weight_safety,
            "box_area": weight_box_area,
            "rpm_error": weight_rpm_error,
            "precision": weight_precision,
            "backlash": weight_backlash,
        }

    st.header("6. 최적화 항목 선택")

    objective_item_label_map = {
        "mass": "Mass, 질량",
        "cost": "Cost, 비용",
        "volume": "Volume, 체적",
        "loss": "Loss, 손실",
        "noise": "Noise, 소음",
        "safety": "Safety, 안전율 패널티",
        "box_area": "Box Area, 박스 면적",
        "rpm_error": "RPM Error, 속도 오차",
        "precision": "Precision, 정밀도",
        "backlash": "Backlash, 백래시",
    }

    available_objective_items = [
        key
        for key in objective_item_label_map.keys()
        if key in objective_weights
    ]

    disabled_objective_labels = st.multiselect(
        "최적화 계산에서 제외할 항목 선택",
        options=[
            objective_item_label_map[key]
            for key in available_objective_items
        ],
        default=[],
        help=(
            "기본값은 전체 항목 활성화입니다. "
            "여기서 선택한 항목만 목적함수 계산에서 비활성화됩니다."
        ),
    )

    label_to_objective_key = {
        objective_item_label_map[key]: key
        for key in available_objective_items
    }

    disabled_objective_items = [
        label_to_objective_key[label]
        for label in disabled_objective_labels
    ]

    objective_weights = apply_disabled_objective_items(
        objective_weights=objective_weights,
        disabled_objective_items=disabled_objective_items,
    )

    active_objective_items = [
        key
        for key, value in objective_weights.items()
        if float(value) != 0.0
    ]

    st.caption(
        "활성 최적화 항목: "
        + (
            ", ".join(
                objective_item_label_map.get(key, key)
                for key in active_objective_items
            )
            if active_objective_items
            else "없음"
        )
    )


    st.header("7. 결과 설정")

    result_count = st.slider(
        "표시할 결과 개수",
        min_value=1,
        max_value=100,
        value=10,
        step=1,
    )

    run_button = st.button(
        "최적화 실행",
        type="primary",
    )


# ============================================================
# 9. 메인 실행부
# ============================================================

st.subheader("입력 모터 DB")

st.dataframe(
    pd.DataFrame(MOTOR_DB),
    use_container_width=True,
)

st.divider()

if run_button:
    if not selected_motors:
        st.error("선택된 모터가 없습니다. 모터 조건을 확인해 주세요.")
        st.stop()

    if not module_candidates:
        st.error("Module 후보가 없습니다. Module 범위를 확인해 주세요.")
        st.stop()

    if not face_width_candidates:
        st.error("Face Width 후보가 없습니다. Face Width 범위를 확인해 주세요.")
        st.stop()

    if min(module_candidates) > max(module_candidates):
        st.error("Module 후보 범위가 올바르지 않습니다.")
        st.stop()

    if min(face_width_candidates) > max(face_width_candidates):
        st.error("Face Width 후보 범위가 올바르지 않습니다.")
        st.stop()

    if int(z_min) > int(z_max):
        st.error("Minimum Tooth Count는 Maximum Tooth Count보다 작거나 같아야 합니다.")
        st.stop()

    if int(min_pinion_teeth) > int(z_max):
        st.error("Minimum Pinion Teeth는 Maximum Tooth Count보다 작거나 같아야 합니다.")
        st.stop()

    if ratio_min > ratio_max:
        st.error("Minimum Total Ratio는 Maximum Total Ratio보다 작거나 같아야 합니다.")
        st.stop()

    if max_face_width_mm <= 0:
        st.error("Maximum Face Width는 0보다 커야 합니다.")
        st.stop()

    with st.spinner("Grid Search 최적화 계산 중입니다..."):
        results = []
        searched_stage_counts = []
        stage_result_counts = {}

        for candidate_stage_count in stage_count_candidates:
            searched_stage_counts.append(candidate_stage_count)

            st.write(f"Stage Count {candidate_stage_count}단 탐색 중...")

            candidate_results = run_grid_search(
                motors=selected_motors,
                gear_type=gear_type,
                material_name=material_name,
                stage_count=int(candidate_stage_count),
                module_candidates=module_candidates,
                face_width_candidates=face_width_candidates,
                z_min=int(z_min),
                z_max=int(z_max),
                min_pinion_teeth=int(min_pinion_teeth),
                max_stage_ratio=float(max_stage_ratio),
                stage_efficiency=float(stage_efficiency),
                external_gearbox_ratio=float(external_gearbox_ratio),
                external_gearbox_efficiency=float(external_gearbox_efficiency),
                helix_angle_deg=float(helix_angle_deg),
                cv=float(cv),
                min_sf_bending=float(min_sf_bending),
                min_sf_contact=float(min_sf_contact),
                target_output_rpm=float(target_output_rpm),
                target_output_torque=float(target_output_torque) if target_output_torque else None,
                ratio_min=float(ratio_min),
                ratio_max=float(ratio_max),
                rpm_tolerance=float(rpm_tolerance),
                max_center_distance_mm=float(max_center_distance_mm),
                max_face_width_mm=float(max_face_width_mm),
                max_box_width_mm=max_box_width_mm,
                max_box_height_mm=max_box_height_mm,
                max_box_area_mm2=max_box_area_mm2,
                objective_weights=objective_weights,
                application_profile=application_profile,
                use_motor_rpm_filter=use_motor_rpm_filter,
                output_torque_min=output_torque_min if use_output_torque_filter else None,
                output_torque_max=output_torque_max if use_output_torque_filter else None,
                motor_rpm_min=float(motor_rpm_min),
                motor_rpm_max=float(motor_rpm_max),
                max_candidates=int(max_candidates),
                max_same_ratio_eval_count=100,
                max_same_ratio_result_count=10,
                ratio_key_ndigits=4,
            )

            stage_result_counts[int(candidate_stage_count)] = len(candidate_results)

            if candidate_results:
                results.extend(candidate_results)

                st.success(
                    f"{candidate_stage_count}단 탐색에서 후보 {len(candidate_results)}개를 찾았습니다."
                )
            else:
                st.warning(
                    f"{candidate_stage_count}단 탐색 결과가 없습니다."
                )

        if results:
            results = sorted(
                results,
                key=lambda x: x.get(
                    "objective_score",
                    x.get(
                        "layout_score",
                        x.get("layout_area_mm2", float("inf")),
                    ),
                ),
            )

            results = results[:int(max_candidates)]

            st.success(
                f"Grid Search 완료: 총 후보 {len(results)}개를 찾았습니다."
            )

            st.info(
                f"탐색한 Stage Count: {searched_stage_counts} / "
                f"단수별 후보 수: {stage_result_counts}"
            )

            st.session_state["grid_search_results"] = results
            st.session_state["searched_stage_counts"] = searched_stage_counts
            st.session_state["stage_result_counts"] = stage_result_counts

            result_df = pd.DataFrame(
                [
                    {
                        "motor_id": r.get("motor_id"),
                        "motor_type": r.get("motor_type"),
                        "gear_type": r.get("gear_type"),
                        "material": r.get("material"),
                        "stage_count": r.get("stage_count"),
                        "total_ratio": r.get("total_ratio"),
                        "output_rpm": r.get("output_rpm"),
                        "rpm_error_pct": r.get("rpm_error_pct"),
                        "output_torque_nm": r.get("output_torque_nm"),
                        "total_efficiency": r.get("total_efficiency"),
                        "loss_w": r.get("loss_w"),
                        "total_mass_kg": r.get("total_mass_kg"),
                        "box_width_mm": r.get("box_width_mm"),
                        "box_height_mm": r.get("box_height_mm"),
                        "box_area_mm2": r.get("box_area_mm2"),
                        "objective_score": r.get("objective_score"),
                    }
                    for r in results
                ]
            )

            st.session_state.summary_df = result_df

            st.subheader("Grid Search 결과")
            st.dataframe(result_df, use_container_width=True)

            best_result = results[0]

            st.subheader("최적 후보")

            col1, col2, col3 = st.columns(3)

            with col1:
                if isinstance(best_result, dict):
                    st.metric(
                        "Stage Count",
                        best_result.get("stage_count", "-")
                    )

                    st.metric(
                        "감속비",
                        best_result.get("total_ratio", best_result.get("ratio", "-"))
                    )

            with col2:
                if isinstance(best_result, dict):
                    st.metric(
                        "출력 RPM",
                        best_result.get("output_rpm", "-")
                    )

                    st.metric(
                        "출력 Torque",
                        best_result.get("output_torque_nm", "-")
                    )

            with col3:
                if isinstance(best_result, dict):
                    st.metric(
                        "Score",
                        best_result.get(
                            "score",
                            best_result.get(
                                "objective_score",
                                best_result.get("total_score", "-")
                            )
                        )
                    )

                    st.metric(
                        "Motor",
                        best_result.get(
                            "motor_name",
                            best_result.get("motor_id", "-")
                        )
                    )

        else:
            st.error(
                "Grid Search 결과가 없습니다. "
                "2단, 3단, 4단 조건을 모두 탐색했지만 유효한 후보를 찾지 못했습니다."
            )

            st.info(
                f"탐색한 Stage Count: {searched_stage_counts} / "
                f"단수별 후보 수: {stage_result_counts}"
            )

            st.session_state["grid_search_results"] = []
            st.session_state["searched_stage_counts"] = searched_stage_counts
            st.session_state["stage_result_counts"] = stage_result_counts
            st.session_state.summary_df = pd.DataFrame()


    if not results:
        st.session_state.results = []
        st.session_state.summary_df = pd.DataFrame()
        st.warning(
            "조건을 만족하는 후보가 없습니다. "
            "허용 오차, 감속비 범위, 안전율, Box 제약 또는 탐색 범위를 완화해 보세요."
        )
        st.stop()

    st.session_state.results = results
    st.session_state.last_result_count = len(results)

    best_stage_count = results[0].get("stage_count", "-") if results else "-"

    st.success(
        f"총 {len(results)}개의 후보를 찾았습니다. "
        f"현재 최상위 후보는 {best_stage_count}단입니다."
    )


if st.session_state.results:
    results = st.session_state.results

    top_results = select_top_results_with_angle_diversity(
        candidate_results=results,
        top_k=int(result_count),
        max_same_angle_signature_count=2,
        max_same_relation_count=20,
        weight_distance=0.0,
    )

    if not top_results:
        st.warning("각도 다양성 조건을 만족하는 상위 결과가 없습니다.")
        st.stop()

    st.session_state.top_results = top_results

    summary_rows = []

    for idx, r in enumerate(top_results, start=1):
        stage_ratio_text = " × ".join(
            [
                f"{s['z_driven']}/{s['z_drive']}={s['stage_ratio']:.3f}"
                for s in r["stage_details"]
            ]
        )

        summary_rows.append(
            {
                "Rank": idx,
                "Motor": r["motor_id"],
                "Motor Type": r["motor_type"],
                "Gear Type": r["gear_type"],
                "Material": r["material"],
                "Stage": r["stage_count"],
                "Stage Ratios": stage_ratio_text,
                "Total Ratio": r["total_ratio"],
                "Output RPM": r["output_rpm"],
                "RPM Error [%]": r["rpm_error_pct"],
                "Output Torque [Nm]": r["output_torque_nm"],
                "Total Efficiency": r["total_efficiency"],
                "Loss [W]": r["loss_w"],
                "Min Bending SF": r["min_bending_sf"],
                "Min Contact SF": r["min_contact_sf"],
                "Total Mass [kg]": r["total_mass_kg"],
                "Total Cost Index": r["total_cost_index"],
                "Box W [mm]": r["box_width_mm"],
                "Box H [mm]": r["box_height_mm"],
                "Box Area [mm²]": r["box_area_mm2"],
                "Noise Score": r["noise_score"],
                "Precision Score": r["precision_score"],
                "Objective Score": r["objective_score"],
            }
        )

    summary_df = pd.DataFrame(summary_rows)
    st.session_state.summary_df = summary_df

    st.subheader("최적화 결과 요약")

    st.dataframe(
        summary_df.style.format(
            {
                "Total Ratio": "{:.3f}",
                "Output RPM": "{:.2f}",
                "RPM Error [%]": "{:.2f}",
                "Output Torque [Nm]": "{:.3f}",
                "Total Efficiency": "{:.3f}",
                "Loss [W]": "{:.3f}",
                "Min Bending SF": "{:.2f}",
                "Min Contact SF": "{:.2f}",
                "Total Mass [kg]": "{:.3f}",
                "Total Cost Index": "{:.3f}",
                "Box W [mm]": "{:.1f}",
                "Box H [mm]": "{:.1f}",
                "Box Area [mm²]": "{:.1f}",
                "Noise Score": "{:.3f}",
                "Precision Score": "{:.3f}",
                "Objective Score": "{:.3f}",
            }
        ),
        use_container_width=True,
    )

    csv = summary_df.to_csv(index=False).encode("utf-8-sig")

    st.download_button(
        label="결과 요약 CSV 다운로드",
        data=csv,
        file_name="motor_gear_optimization_summary.csv",
        mime="text/csv",
    )

    st.divider()

    st.subheader("상세 결과 선택")

    selected_rank = st.selectbox(
        "상세 확인할 Rank 선택",
        list(range(1, len(top_results) + 1)),
        index=0,
        key="selected_detail_rank",
    )

    selected_result = top_results[selected_rank - 1]

    col1, col2, col3, col4 = st.columns(4)

    with col1:
        st.metric("Motor", selected_result["motor_id"])
        st.metric("Total Ratio", f"{selected_result['total_ratio']:.3f}")
        st.metric("Output RPM", f"{selected_result['output_rpm']:.2f}")

    with col2:
        st.metric("Output Torque", f"{selected_result['output_torque_nm']:.3f} Nm")
        st.metric("Total Efficiency", f"{selected_result['total_efficiency']:.3f}")
        st.metric("Loss", f"{selected_result['loss_w']:.3f} W")

    with col3:
        st.metric("Min Bending SF", f"{selected_result['min_bending_sf']:.2f}")
        st.metric("Min Contact SF", f"{selected_result['min_contact_sf']:.2f}")
        st.metric("Mass", f"{selected_result['total_mass_kg']:.3f} kg")

    with col4:
        st.metric("Box Width", f"{selected_result['box_width_mm']:.1f} mm")
        st.metric("Box Height", f"{selected_result['box_height_mm']:.1f} mm")
        st.metric("Box Area", f"{selected_result['box_area_mm2']:.1f} mm²")

    st.subheader("선택 후보 기본 정보")

    basic_info = {
        "motor_id": selected_result["motor_id"],
        "motor_type": selected_result["motor_type"],
        "gear_type": selected_result["gear_type"],
        "material": selected_result["material"],
        "stage_count": selected_result["stage_count"],
        "module_mm": selected_result["module_mm"],
        "face_width_mm": selected_result["face_width_mm"],
        "helix_angle_deg": selected_result["helix_angle_deg"],
        "external_gearbox_ratio": selected_result["external_gearbox_ratio"],
        "total_stage_ratio": selected_result["total_stage_ratio"],
        "total_ratio": selected_result["total_ratio"],
        "gear_total_efficiency": selected_result["gear_total_efficiency"],
        "total_efficiency": selected_result["total_efficiency"],
        "objective_score": selected_result["objective_score"],
    }

    st.json(basic_info)

    st.subheader("단별 상세 계산 결과")

    stage_df = pd.DataFrame(selected_result["stage_details"])

    st.dataframe(
        stage_df.style.format(
            {
                "stage_ratio": "{:.4f}",
                "module_mm": "{:.3f}",
                "face_width_mm": "{:.2f}",
                "d_drive_mm": "{:.2f}",
                "d_driven_mm": "{:.2f}",
                "center_distance_mm": "{:.2f}",
                "rpm_in": "{:.2f}",
                "rpm_out": "{:.2f}",
                "torque_in_nm": "{:.4f}",
                "torque_out_nm": "{:.4f}",
                "pitch_velocity_m_s": "{:.4f}",
                "kv": "{:.4f}",
                "tangential_force_n": "{:.2f}",
                "lewis_y": "{:.4f}",
                "bending_stress_mpa": "{:.2f}",
                "contact_stress_mpa": "{:.2f}",
                "sf_bending": "{:.2f}",
                "sf_contact": "{:.2f}",
                "gear_mass_kg": "{:.5f}",
            }
        ),
        use_container_width=True,
    )

    stage_csv = stage_df.to_csv(index=False).encode("utf-8-sig")

    st.download_button(
        label="선택 후보 단별 상세 CSV 다운로드",
        data=stage_csv,
        file_name=f"stage_detail_rank_{selected_rank}.csv",
        mime="text/csv",
    )

    st.divider()

    st.subheader("시각화")

    tab1, tab2, tab3, tab4, tab5 = st.tabs(
        [
            "Torque / RPM",
            "Layout",
            "Summary",
            "Stage Details",
            "Raw Data",
        ]
    )

    with tab1:
        show_half_width_pyplot(
            plot_stage_torque_rpm(selected_result["stage_details"]),
            center=False,
        )

    with tab2:
        show_half_width_pyplot(
            plot_layout(selected_result),
            center=False,
        )

    with tab3:
        st.subheader("Selected Motor")

        selected_motor = selected_result.get("motor", {})

        if selected_motor:
            motor_df = pd.DataFrame([selected_motor])
            st.dataframe(motor_df, use_container_width=True)
        else:
            st.info("No selected motor data available.")

        st.subheader("Optimization Summary")

        summary_data = {
            "Motor ID": selected_motor.get("motor_id", "-"),
            "Total Gear Ratio": selected_result.get("total_ratio", "-"),
            "Target Gear Ratio": selected_result.get("target_ratio", "-"),
            "Output Torque [Nm]": selected_result.get("output_torque_nm", "-"),
            "Output Speed [rpm]": selected_result.get("output_rpm", "-"),
            "Layout Width [mm]": selected_result.get("layout", {})
            .get("bbox", {})
            .get("width", "-"),
            "Layout Height [mm]": selected_result.get("layout", {})
            .get("bbox", {})
            .get("height", "-"),
        }

        st.table(pd.DataFrame([summary_data]).T.rename(columns={0: "Value"}))

    with tab4:
        st.subheader("Stage Details")

        stage_details = selected_result.get("stage_details", [])

        if stage_details:
            stage_df = pd.DataFrame(stage_details)
            st.dataframe(stage_df, use_container_width=True)
        else:
            st.info("No stage detail data available.")

    with tab5:
        st.subheader("Raw Optimization Result")
        st.json(selected_result)

    st.subheader("Stage Center Distance / Angle")

    stage_distance_rows = get_stage_distance_table(selected_result)

    if stage_distance_rows:
        stage_distance_df = pd.DataFrame(stage_distance_rows)
        st.dataframe(stage_distance_df, use_container_width=True, hide_index=True)
    else:
        st.info("No stage distance data available.")

    st.subheader("Motor Axis to Output Axis")

    layout = selected_result.get("layout", {})
    motor_to_output = layout.get("motor_to_output", {})

    motor_output_df = pd.DataFrame(
        [
            {
                "dx_mm": motor_to_output.get("dx_mm", "-"),
                "dy_mm": motor_to_output.get("dy_mm", "-"),
                "distance_mm": motor_to_output.get("distance_mm", "-"),
            }
        ]
    )

    st.dataframe(motor_output_df, use_container_width=True, hide_index=True)

    st.divider()

    st.subheader("최적 Layout Angle 정보")

    layout_info = {
        "angles_deg": selected_result.get(
            "best_angles_deg",
            selected_result.get("layout", {}).get("stage_angles_deg", []),
        ),
        "box_width_mm": selected_result.get("box_width_mm", "-"),
        "box_height_mm": selected_result.get("box_height_mm", "-"),
        "box_area_mm2": selected_result.get("box_area_mm2", "-"),
        "motor_to_output": selected_result.get("layout", {}).get("motor_to_output", {}),
    }

    st.json(layout_info)

    circle_df = pd.DataFrame(selected_result["layout"]["circles"])

    st.dataframe(
        circle_df.style.format(
            {
                "x": "{:.2f}",
                "y": "{:.2f}",
                "r": "{:.2f}",
            }
        ),
        use_container_width=True,
    )

else:
    st.info("좌측 Sidebar에서 조건을 설정한 뒤, **최적화 실행** 버튼을 누르세요.")


'''
1. 모터 DB
2. 소재 DB
3. 유틸 함수
4. 기어 형상/강도 계산 함수
5. Layout Box 계산 함수
6. 후보 평가 함수
7. Grid Search 함수
8. 시각화 함수
9. Streamlit Sidebar 입력부
10. 메인 실행부
11. 상세 결과 및 시각화 출력부
'''
