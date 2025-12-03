# High-Bypass Turbofan Cycle Optimization

This project performs a parametric design optimization for a **High-Bypass Turbofan Engine**.

Using MATLAB's constrained nonlinear optimization solver (`fmincon`), the script determines the optimal thermodynamic cycle parameters to minimize **Specific Fuel Consumption (SFC)** while satisfying a strict Thrust requirement.

## 🎯 Project Objective

To find the design point that yields the maximum fuel efficiency (Minimum SFC) under the following operational constraint:
* **Target Thrust:** $T \geq 100 \text{ kN}$ 

## ⚙️ Design Variables & Bounds

The optimization algorithm manipulates 5 key engine cycle parameters within feasible engineering limits:

| Variable | Description | Lower Bound | Upper Bound |
| :--- | :--- | :--- | :--- |
| $\dot{m}_0$ | Mass Flow Rate (kg/s) | 200 | 600 |
| $\alpha$ | Bypass Ratio (BPR) | 1 | 14 |
| $\pi_f$ | Fan Pressure Ratio | 1.3 | 4 |
| $\pi_c$ | Compressor Pressure Ratio | 30 | 60 |
| $T_{t4}$ | Turbine Inlet Temp (K) | 1600 | 2100 |

## 🧠 Methodology

* **Thermodynamic Model:** A 0-D parametric cycle analysis (Real Cycle) including component efficiencies ($\eta_c, \eta_t, \eta_b$, etc.) and polytropic losses.
* [cite_start]**Solver:** MATLAB `fmincon` with the **Sequential Quadratic Programming (SQP)** algorithm.
* **Objective Function:**
    $$\min f(x) = SFC = \frac{\dot{m}_f}{\text{Thrust}}$$

## 📊 Results

The optimization converged to the following design point:

* **Min SFC:** $0.4435 \text{ (kg/N/h)}$ (approx. scaled)
* **Thrust:** $100.00 \text{ kN}$ (Active Constraint)
* **Turbine Inlet Temp:** $2040 \text{ K}$
* **Bypass Ratio:** $2.39$

## 🚀 How to Run

1.  Clone the repository.
2.  Run `Turbofan_Optimization.m` in MATLAB.
3.  The script will output the iteration history and final optimized parameters to the command window.

## 👤 Author

**Kivanc Apaydin**
*Aeronautical Engineering Student Project*
