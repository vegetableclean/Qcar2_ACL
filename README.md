# ACL QCar Testbed

A repository for building and maintaining a reproducible autonomous-vehicle testbed in the **Autonomic Computing Lab (ACL), University of Arizona**. The testbed supports rapid experimentation, teaching, and demos on Quanser QCar platforms.

> **Attribution:** Portions of this codebase originate from **Quanser** examples and have been **modified by the Autonomic Computing Lab (ACL)**.

---

## Goals
- Provide a **reliable, reusable codebase** validated on ACL’s hardware.
- Enable **sensor fusion, perception, control, and V2V/V2I** experiments.
- Keep setup steps **documented and automatable** so new members can get started quickly.

---

## Testbed Hardware (Current)
- **QCar 2** × 2 units
- **QCar (Gen-1)** × 1 unit
- **Miniature road map / track**
- **Traffic lights & road signs** (scaled)



---

## Team
- **Chieh Tsai** — PhD Student, ECE Department, University of Arizona (project lead)
- **Prof. Salim Hariri** — Faculty Advisor, Autonomic Computing Lab



---

## Tech Stack
- **Languages:** MATLAB / Simulink, Python
- **Operating Systems:** Windows and Linux (PC side)



---

## Quick Start — Quanser Software & Docs (PC)

### 🔗 Resource Links
- **Student Resources**  
  https://quanserinc.box.com/shared/static/9tob8mj31lkw6r8z4huh26behctlae4q.zip

- **Instructor Resources** *(restricted to instructors — do not share with students)*  
  https://quanserinc.box.com/shared/static/s8lrc6fxy6g10eb63b7u9m7ifxrfmxgy.zip

### 📄 Installation (Windows PC)
1. Unzip the package(s) above. Open the folder and **read `readme_user.txt`** for details on Simulink and/or Python configurations.
2. Run the following batch files **in order**:
   - `step_1_select_resources.bat` — choose which resources to use
   - `step_2_download_resources_instructor.bat` — download the selected packages
3. Navigate to `Documents\Quanser\1_setup` and run:
   - `configure_matlab.bat`
   - `configure_python.bat`
4. **Restart your PC** to apply environment-variable updates.

**Troubleshooting:** If you hit errors, capture the console output and open an issue with your OS version, MATLAB version, and Python version.

---

## Repository Layout
> _Suggested structure; adjust as the project evolves._
## Acknowledgments
- **Quanser** for QCar platforms, examples, and educational materials.
- **Autonomic Computing Lab (ACL)**, University of Arizona.

---

## Citation (optional)
If this repository supports a publication, please cite it appropriately. Example BibTeX:

```bibtex
@misc{acl_qcar_testbed,
  title  = {ACL QCar Testbed},
  author = {Tsai, Chieh and Hariri, Salim and ACL Contributors},
  year   = {2025},
  howpublished = {\url{https://github.com/<org>/<repo>}},
  note   = {Quanser QCar testbed code and docs}
}
