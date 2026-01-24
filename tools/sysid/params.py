"""
Parameter Management for System Identification
システム同定用パラメータ管理

Load, save, merge, diff, and validate parameters.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from .defaults import DEFAULT_PARAMS, get_default_params


def load_params(filepath: str | Path) -> Dict[str, Any]:
    """Load parameters from YAML or JSON file
    YAML/JSONファイルからパラメータを読み込み

    Args:
        filepath: Path to parameter file

    Returns:
        Parameter dictionary

    Raises:
        FileNotFoundError: If file doesn't exist
        ValueError: If file format is unsupported
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Parameter file not found: {filepath}")

    with open(path, 'r') as f:
        if path.suffix in ['.yaml', '.yml']:
            data = yaml.safe_load(f)
        elif path.suffix == '.json':
            data = json.load(f)
        else:
            raise ValueError(f"Unsupported file format: {path.suffix}")

    return data or {}


def save_params(
    params: Dict[str, Any],
    filepath: str | Path,
    format: str = 'yaml',
    metadata: Optional[Dict[str, Any]] = None,
) -> None:
    """Save parameters to file
    パラメータをファイルに保存

    Args:
        params: Parameter dictionary
        filepath: Output file path
        format: Output format ('yaml' or 'json')
        metadata: Optional metadata to include (timestamp, method, etc.)
    """
    path = Path(filepath)
    path.parent.mkdir(parents=True, exist_ok=True)

    # Add metadata if provided
    output = dict(params)
    if metadata:
        output['_metadata'] = metadata

    with open(path, 'w') as f:
        if format == 'json' or path.suffix == '.json':
            json.dump(output, f, indent=2)
        else:
            yaml.dump(output, f, default_flow_style=False, sort_keys=False)


def merge_params(
    base: Dict[str, Any],
    updates: Dict[str, Any],
    deep: bool = True,
) -> Dict[str, Any]:
    """Merge two parameter dictionaries
    2つのパラメータ辞書をマージ

    Args:
        base: Base parameters
        updates: Updates to apply
        deep: If True, merge nested dicts; if False, replace entirely

    Returns:
        Merged dictionary
    """
    import copy
    result = copy.deepcopy(base)

    for key, value in updates.items():
        if deep and key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_params(result[key], value, deep=True)
        else:
            result[key] = copy.deepcopy(value)

    return result


def diff_params(
    params1: Dict[str, Any],
    params2: Dict[str, Any],
    prefix: str = "",
) -> List[Tuple[str, Any, Any]]:
    """Compare two parameter dictionaries
    2つのパラメータ辞書を比較

    Args:
        params1: First parameter dict
        params2: Second parameter dict
        prefix: Key prefix for nested dicts

    Returns:
        List of (key, value1, value2) tuples for differing values
    """
    diffs = []

    all_keys = set(params1.keys()) | set(params2.keys())

    for key in sorted(all_keys):
        full_key = f"{prefix}.{key}" if prefix else key
        v1 = params1.get(key)
        v2 = params2.get(key)

        if isinstance(v1, dict) and isinstance(v2, dict):
            diffs.extend(diff_params(v1, v2, full_key))
        elif v1 != v2:
            diffs.append((full_key, v1, v2))

    return diffs


def validate_params(params: Dict[str, Any]) -> List[str]:
    """Validate parameter values for physical consistency
    物理的整合性をチェック

    Checks:
    - Hover thrust balance: m×g ≈ 4×Ct×ω²
    - Inertia ordering: Izz > Iyy > Ixx (typical X-quad)
    - κ consistency: κ = Cq/Ct
    - Positive values where required

    Args:
        params: Parameter dictionary

    Returns:
        List of warning messages (empty if all OK)
    """
    warnings = []

    # Helper to extract value from nested structure
    def get_val(d: Dict, *keys) -> Optional[float]:
        current = d
        for k in keys:
            if not isinstance(current, dict):
                return None
            current = current.get(k)
            if current is None:
                return None
        if isinstance(current, dict) and 'value' in current:
            return current['value']
        return current if isinstance(current, (int, float)) else None

    # Get values
    mass = get_val(params, 'mass') or get_val(params, 'mass', 'value')
    Ixx = get_val(params, 'inertia', 'Ixx') or get_val(params, 'Ixx')
    Iyy = get_val(params, 'inertia', 'Iyy') or get_val(params, 'Iyy')
    Izz = get_val(params, 'inertia', 'Izz') or get_val(params, 'Izz')
    Ct = get_val(params, 'motor', 'Ct') or get_val(params, 'Ct')
    Cq = get_val(params, 'motor', 'Cq') or get_val(params, 'Cq')
    kappa = get_val(params, 'motor', 'kappa') or get_val(params, 'kappa')

    g = 9.80665

    # Check hover thrust balance
    if mass and Ct:
        hover_thrust = mass * g / 4  # per motor
        hover_omega = (hover_thrust / Ct) ** 0.5
        # Reasonable hover rpm: 20000-40000 (2094-4189 rad/s)
        if hover_omega < 1000 or hover_omega > 6000:
            warnings.append(
                f"Hover angular velocity ({hover_omega:.0f} rad/s) seems unusual. "
                f"Check mass ({mass} kg) and Ct ({Ct:.2e})."
            )

    # Check inertia ordering (typical for X-quad: Izz > Iyy >= Ixx)
    if Ixx and Iyy and Izz:
        if not (Izz > Iyy and Izz > Ixx):
            warnings.append(
                f"Unusual inertia ordering: Ixx={Ixx:.2e}, Iyy={Iyy:.2e}, Izz={Izz:.2e}. "
                f"For X-quads, typically Izz > Iyy > Ixx."
            )

    # Check κ consistency
    if Ct and Cq and kappa:
        expected_kappa = Cq / Ct
        error_pct = abs(kappa - expected_kappa) / expected_kappa * 100
        if error_pct > 5:
            warnings.append(
                f"κ inconsistency: κ={kappa:.4e} but Cq/Ct={expected_kappa:.4e} "
                f"(error: {error_pct:.1f}%)"
            )

    # Check positive values
    positive_checks = [
        ('mass', mass),
        ('Ixx', Ixx),
        ('Iyy', Iyy),
        ('Izz', Izz),
        ('Ct', Ct),
        ('Cq', Cq),
    ]
    for name, val in positive_checks:
        if val is not None and val <= 0:
            warnings.append(f"{name} must be positive (got {val})")

    return warnings


def flatten_params(params: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """Flatten nested parameter dict to dot-notation keys
    ネストしたパラメータ辞書をドット記法のキーにフラット化

    Args:
        params: Nested parameter dict
        prefix: Key prefix

    Returns:
        Flat dictionary
    """
    items = {}
    for k, v in params.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            if 'value' in v:
                # Extract value from {value: x, unit: y} structure
                items[key] = v['value']
            else:
                items.update(flatten_params(v, key))
        else:
            items[key] = v
    return items


def unflatten_params(flat: Dict[str, Any]) -> Dict[str, Any]:
    """Unflatten dot-notation keys to nested dict
    ドット記法のキーをネストした辞書に展開

    Args:
        flat: Flat dictionary with dot-notation keys

    Returns:
        Nested dictionary
    """
    result = {}
    for key, value in flat.items():
        parts = key.split('.')
        current = result
        for part in parts[:-1]:
            if part not in current:
                current[part] = {}
            current = current[part]
        current[parts[-1]] = value
    return result


def create_result_params(
    estimated: Dict[str, float],
    reference: Optional[Dict[str, float]] = None,
    method: str = "unknown",
    log_file: Optional[str] = None,
    fit_quality: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """Create a parameter result structure for output
    出力用のパラメータ結果構造を作成

    Args:
        estimated: Estimated parameter values
        reference: Reference values for comparison
        method: Identification method name
        log_file: Source log file
        fit_quality: Fit quality metrics (r_squared, etc.)

    Returns:
        Structured result dictionary
    """
    result = {
        'method': method,
        'timestamp': datetime.now().isoformat(),
    }

    if log_file:
        result['log_file'] = str(log_file)

    result['estimated'] = {}
    for key, val in estimated.items():
        if isinstance(val, float):
            result['estimated'][key] = val
        else:
            result['estimated'][key] = float(val)

    if reference:
        result['reference'] = {}
        result['comparison'] = {}
        for key, est_val in estimated.items():
            if key in reference:
                ref_val = reference[key]
                result['reference'][key] = ref_val
                if ref_val != 0:
                    error_pct = abs(est_val - ref_val) / abs(ref_val) * 100
                    result['comparison'][key] = {
                        'error_percent': round(error_pct, 2),
                        'status': 'OK' if error_pct < 20 else 'CHECK',
                    }

    if fit_quality:
        result['fit_quality'] = fit_quality

    return result


def export_to_c_header(
    params: Dict[str, Any],
    output_path: str | Path,
    header_guard: str = "SYSID_PARAMS_H",
) -> None:
    """Export parameters to C header file
    パラメータをCヘッダーファイルにエクスポート

    Args:
        params: Parameter dictionary
        output_path: Output .h file path
        header_guard: Header guard macro name
    """
    path = Path(output_path)
    flat = flatten_params(params)

    lines = [
        f"#ifndef {header_guard}",
        f"#define {header_guard}",
        "",
        "// Auto-generated from system identification",
        f"// Generated: {datetime.now().isoformat()}",
        "",
    ]

    # Convert to C macros
    for key, value in sorted(flat.items()):
        if isinstance(value, (int, float)):
            macro_name = key.upper().replace('.', '_')
            if isinstance(value, float):
                lines.append(f"#define SYSID_{macro_name} {value:.6e}f")
            else:
                lines.append(f"#define SYSID_{macro_name} {value}")

    lines.extend([
        "",
        f"#endif // {header_guard}",
        "",
    ])

    with open(path, 'w') as f:
        f.write('\n'.join(lines))
