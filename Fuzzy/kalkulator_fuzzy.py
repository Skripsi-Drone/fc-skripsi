# @title Fuzzy Membership Generator (Combined)
# @markdown Jalankan sel ini. Program akan berjalan dalam mode Interaktif secara default.
# @markdown Hasil file (.h dan .png) akan muncul di folder **Files** (ikon folder di kiri).

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json
import sys
import os

# ============================================================================
# BAGIAN 1: CORE CLASSES & LOGIC (Dari fuzzy_membership_calculator.py)
# ============================================================================

@dataclass
class MembershipFunction:
    """Class untuk menyimpan parameter membership function"""
    name: str
    mf_type: str  # 'triangle' atau 'trapezoid'
    params: List[float]  # [a, b, c] untuk triangle, [a, b, c, d] untuk trapezoid
    
    def calculate(self, x: float) -> float:
        if self.mf_type == 'triangle':
            return self._triangle(x)
        elif self.mf_type == 'trapezoid':
            return self._trapezoid(x)
        else:
            raise ValueError(f"Unknown membership type: {self.mf_type}")
    
    def _triangle(self, x: float) -> float:
        a, b, c = self.params
        if x <= a: return 0.0
        elif a < x <= b: return (x - a) / (b - a) if (b - a) != 0 else 1.0
        elif b < x <= c: return (c - x) / (c - b) if (c - b) != 0 else 1.0
        else: return 0.0
    
    def _trapezoid(self, x: float) -> float:
        a, b, c, d = self.params
        if x <= a: return 0.0
        elif a < x <= b: return (x - a) / (b - a) if (b - a) != 0 else 1.0
        elif b < x <= c: return 1.0
        elif c < x <= d: return (d - x) / (d - c) if (d - c) != 0 else 1.0
        else: return 0.0
    
    def get_arduino_code(self) -> str:
        if self.mf_type == 'triangle':
            return f'FuzzySet *{self.name} = new FuzzySet({self.params[0]}, {self.params[1]}, {self.params[1]}, {self.params[2]});'
        else:
            return f'FuzzySet *{self.name} = new FuzzySet({self.params[0]}, {self.params[1]}, {self.params[2]}, {self.params[3]});'

class FuzzyMembershipGenerator:
    def __init__(self, range_min: float, range_max: float, num_memberships: int = 3):
        self.range_min = range_min
        self.range_max = range_max
        self.num_memberships = num_memberships
        self.total_range = range_max - range_min
        
    def generate_triangle_auto(self, overlap_percent: float = 30.0, names: Optional[List[str]] = None) -> List[MembershipFunction]:
        if names is None:
            if self.num_memberships == 3: names = ['NEG', 'ZERO', 'POS']
            elif self.num_memberships == 5: names = ['NL', 'NS', 'ZERO', 'PS', 'PL']
            else: names = [f'MF{i+1}' for i in range(self.num_memberships)]
        
        overlap_factor = overlap_percent / 100.0
        base_width = self.total_range / (self.num_memberships - overlap_factor * (self.num_memberships - 1))
        overlap_width = base_width * overlap_factor
        step = base_width - overlap_width
        
        memberships = []
        for i in range(self.num_memberships):
            if i == 0:
                a, b, c = self.range_min, self.range_min + base_width / 2, self.range_min + base_width
            elif i == self.num_memberships - 1:
                a, b, c = self.range_max - base_width, self.range_max - base_width / 2, self.range_max
            else:
                center = self.range_min + step * i + base_width / 2
                a, b, c = center - base_width / 2, center, center + base_width / 2
            
            memberships.append(MembershipFunction(names[i], 'triangle', [round(a, 2), round(b, 2), round(c, 2)]))
        return memberships
    
    def generate_trapezoid_auto(self, overlap_percent: float = 25.0, flat_top_percent: float = 20.0, names: Optional[List[str]] = None) -> List[MembershipFunction]:
        if names is None:
            if self.num_memberships == 3: names = ['NEG', 'ZERO', 'POS']
            elif self.num_memberships == 5: names = ['NL', 'NS', 'ZERO', 'PS', 'PL']
            else: names = [f'MF{i+1}' for i in range(self.num_memberships)]
        
        overlap_factor = overlap_percent / 100.0
        flat_factor = flat_top_percent / 100.0
        base_width = self.total_range / (self.num_memberships - overlap_factor * (self.num_memberships - 1))
        overlap_width = base_width * overlap_factor
        step = base_width - overlap_width
        flat_width = base_width * flat_factor
        slope_width = (base_width - flat_width) / 2
        
        memberships = []
        for i in range(self.num_memberships):
            if i == 0:
                a, b, c, d = self.range_min, self.range_min, self.range_min + flat_width, self.range_min + flat_width + slope_width * 2
            elif i == self.num_memberships - 1:
                a, b, c, d = self.range_max - flat_width - slope_width * 2, self.range_max - flat_width, self.range_max, self.range_max
            else:
                center = self.range_min + step * i + base_width / 2
                a = center - flat_width / 2 - slope_width
                b = center - flat_width / 2
                c = center + flat_width / 2
                d = center + flat_width / 2 + slope_width
            
            memberships.append(MembershipFunction(names[i], 'trapezoid', [round(a, 2), round(b, 2), round(c, 2), round(d, 2)]))
        return memberships

    def generate_custom(self, custom_params: List[dict]) -> List[MembershipFunction]:
        memberships = []
        for param in custom_params:
            memberships.append(MembershipFunction(param['name'], param['type'], param['params']))
        return memberships

class FuzzyVisualizer:
    @staticmethod
    def plot_memberships(memberships: List[MembershipFunction], range_min: float, range_max: float, title: str = "Fuzzy Membership Functions", save_path: Optional[str] = None):
        x = np.linspace(range_min, range_max, 1000)
        fig, ax = plt.subplots(figsize=(12, 6))
        colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown', 'pink']
        
        for i, mf in enumerate(memberships):
            y = np.array([mf.calculate(xi) for xi in x])
            color = colors[i % len(colors)]
            ax.plot(x, y, label=f'{mf.name} ({mf.mf_type})', color=color, linewidth=2)
            ax.fill_between(x, 0, y, alpha=0.2, color=color)
        
        ax.set_xlabel('Input Value', fontsize=12, fontweight='bold')
        ax.set_ylabel('Degree of Membership (μ)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_ylim(-0.05, 1.1)
        ax.axhline(y=0, color='k', linewidth=0.5)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✅ Plot saved to: {save_path}")
        
        plt.show() # Added for Colab
        return fig
    
    @staticmethod
    def plot_activation(memberships: List[MembershipFunction], range_min: float, range_max: float, test_value: float, title: str = "Membership Activation"):
        x = np.linspace(range_min, range_max, 1000)
        fig, ax = plt.subplots(figsize=(12, 6))
        colors = ['blue', 'green', 'red', 'purple', 'orange']
        
        print(f"\n{'='*60}")
        print(f"ACTIVATION TEST: Input = {test_value}")
        print(f"{'='*60}")
        
        for i, mf in enumerate(memberships):
            y = np.array([mf.calculate(xi) for xi in x])
            activation = mf.calculate(test_value)
            color = colors[i % len(colors)]
            ax.plot(x, y, label=f'{mf.name}', color=color, linewidth=2, alpha=0.7)
            ax.fill_between(x, 0, y, alpha=0.1, color=color)
            
            if activation > 0:
                ax.plot([test_value], [activation], 'o', color=color, markersize=10, label=f'{mf.name} μ={activation:.3f}')
                ax.plot([range_min, test_value], [activation, activation], '--', color=color, alpha=0.5)
                ax.plot([test_value, test_value], [0, activation], '--', color=color, alpha=0.5)
                print(f"{mf.name:10s}: μ = {activation:.4f} ({activation*100:.1f}%)")
        
        ax.axvline(x=test_value, color='red', linewidth=2, linestyle=':', label=f'Input = {test_value}')
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.1)
        plt.tight_layout()
        plt.show() # Added for Colab
        return fig

def export_to_arduino(memberships: List[MembershipFunction], variable_name: str, output_file: str):
    code = []
    code.append(f"// Fuzzy Membership Functions for {variable_name}")
    code.append(f"// Auto-generated by Fuzzy Membership Calculator")
    code.append("")
    code.append(f"FuzzyInput *{variable_name} = new FuzzyInput(1);")
    code.append("")
    
    for mf in memberships:
        code.append(mf.get_arduino_code())
        code.append(f"{variable_name}->addFuzzySet({mf.name});")
        code.append("")
    
    code_str = "\n".join(code)
    with open(output_file, 'w') as f:
        f.write(code_str)
    print(f"✅ Arduino code exported to: {output_file}")
    return code_str

def print_membership_info(memberships: List[MembershipFunction]):
    print("\n" + "="*80)
    print("MEMBERSHIP FUNCTION DETAILS")
    print("="*80)
    for mf in memberships:
        print(f"\n{mf.name} ({mf.mf_type.upper()}):")
        print(f"  Params: {mf.params}")

# ============================================================================
# BAGIAN 2: INTERACTIVE INTERFACE (Dari fuzzy_calculator_interactive.py)
# ============================================================================

def get_float_input(prompt: str, default: Optional[float] = None) -> float:
    while True:
        try:
            if default is not None:
                user_input = input(f"{prompt} [default: {default}]: ").strip()
                if user_input == "": return default
            else:
                user_input = input(f"{prompt}: ").strip()
            return float(user_input)
        except ValueError:
            print("❌ Input tidak valid! Masukkan angka yang benar.")

def get_int_input(prompt: str, default: Optional[int] = None, min_val: int = 1, max_val: int = 10) -> int:
    while True:
        try:
            if default is not None:
                user_input = input(f"{prompt} [{min_val}-{max_val}] [default: {default}]: ").strip()
                if user_input == "": return default
            else:
                user_input = input(f"{prompt} [{min_val}-{max_val}]: ").strip()
            value = int(user_input)
            if min_val <= value <= max_val: return value
            else: print(f"❌ Nilai harus antara {min_val} dan {max_val}!")
        except ValueError:
            print("❌ Input tidak valid! Masukkan angka bulat.")

def get_choice(prompt: str, options: List[str]) -> str:
    print(f"\n{prompt}")
    for i, opt in enumerate(options, 1):
        print(f"  {i}. {opt}")
    while True:
        try:
            choice = int(input(f"Pilih (1-{len(options)}): "))
            if 1 <= choice <= len(options): return options[choice - 1]
            else: print(f"❌ Pilih angka 1-{len(options)}!")
        except ValueError:
            print("❌ Input tidak valid!")

def main_interactive():
    print("="*80)
    print("FUZZY MEMBERSHIP FUNCTION CALCULATOR (COLAB EDITION)")
    print("="*80)
    
    # Input Range
    print("\n📊 STEP 1: INPUT RANGE")
    range_min = get_float_input("Range MINIMUM")
    range_max = get_float_input("Range MAXIMUM")
    
    if range_min >= range_max:
        print("❌ Error: Range minimum harus lebih kecil dari maximum!")
        return
    
    # Input MF Parameters
    print("\n📊 STEP 2: KONFIGURASI")
    num_mf = get_int_input("Jumlah membership functions", default=3, min_val=2, max_val=7)
    mf_type = get_choice("Pilih tipe membership function:", ["Triangle", "Trapezoid"])
    
    overlap = get_float_input("\nOverlap percentage (0-50)", default=30.0)
    
    # Generate
    if num_mf == 3: names = ['NEG', 'ZERO', 'POS']
    elif num_mf == 5: names = ['NL', 'NS', 'ZERO', 'PS', 'PL']
    else: names = [f'MF{i+1}' for i in range(num_mf)]
    
    gen = FuzzyMembershipGenerator(range_min, range_max, num_mf)
    
    if mf_type == "Triangle":
        memberships = gen.generate_triangle_auto(overlap_percent=overlap, names=names)
    elif mf_type == "Trapezoid":
        flat_top = get_float_input("Flat top percentage (10-40)", default=20.0)
        memberships = gen.generate_trapezoid_auto(overlap_percent=overlap, flat_top_percent=flat_top, names=names)
    
    print_membership_info(memberships)
    
    # Visualize & Save
    variable_name = input("\nNama variabel (contoh: errorRoll): ").strip()
    if not variable_name: variable_name = "fuzzyInput"
    
    # Use /content/ for Colab compatibility
    vis = FuzzyVisualizer()
    print("\n📈 VISUALISASI:")
    vis.plot_memberships(memberships, range_min, range_max, 
                        title=f"Membership Functions: {variable_name}",
                        save_path=f"/content/{variable_name}_memberships.png")
    
    # Test Activation
    do_test = input("\nMau test dengan nilai input tertentu? (y/n) [n]: ").strip().lower()
    if do_test == 'y':
        test_val = get_float_input(f"Masukkan nilai test ({range_min} s/d {range_max})")
        vis.plot_activation(memberships, range_min, range_max, test_val,
                           title=f"Activation Test: {variable_name} = {test_val}")
        plt.savefig(f"/content/{variable_name}_activation.png")

    # Export
    export_to_arduino(memberships, variable_name, f"/content/{variable_name}_fuzzy.h")
    print(f"\n✅ Selesai! Cek folder 'Files' di sidebar kiri untuk mengunduh {variable_name}_fuzzy.h")

def run_hardcoded_demo():
    print("Menjalankan Hardcoded Demo...")
    gen_roll = FuzzyMembershipGenerator(range_min=-150, range_max=45, num_memberships=3)
    mf_roll_trap = gen_roll.generate_trapezoid_auto(overlap_percent=25.0, flat_top_percent=20.0)
    
    vis = FuzzyVisualizer()
    vis.plot_memberships(mf_roll_trap, -150, 45, 
                        title="DEMO: Error Roll - Trapezoid",
                        save_path="/content/demo_roll.png")
    
    export_to_arduino(mf_roll_trap, "demoRoll", "/content/demo_roll.h")
    print("Demo selesai. Cek folder output.")

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    print("Pilih Mode:")
    print("1. Interactive (Input nilai sendiri)")
    print("2. Demo (Hardcoded example: Error Roll)")
    
    try:
        mode = input("Pilihan [1]: ").strip()
    except:
        mode = "1" # Default if input fails
        
    if mode == "2":
        run_hardcoded_demo()
    else:
        main_interactive()