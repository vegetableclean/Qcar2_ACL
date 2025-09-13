
---

## Usage
- Start with the **docs/** and **config/** templates to match your lab setup.
- MATLAB/Simulink and Python modules are **modular**—run independently or as part of larger experiments.
- For reproducibility, prefer **config-driven** parameters over hard-coded values.

---

## Development Guidelines
- Branch names: `feat/<name>`, `fix/<name>`, `exp/<name>`.
- Use **Issues** for bugs/setup problems; attach logs/screenshots.
- Keep this **README** and **docs/** updated when hardware/workflows change.

---

## Requirements (typical)
- **MATLAB/Simulink** (version per Quanser guidelines)
- **Python 3.x** (see `python/requirements.txt`)
- **Quanser drivers/tooling** per installation steps above

> Exact versions may vary — see module-specific READMEs.

---

## Safety & Lab Etiquette
- Always **power down** QCars before reconnecting hardware.
- Keep the **track area clear**; announce tests before running vehicles.
- Log incidents (collisions, anomalies) in `docs/lab-log.md`.

---

## License & Attribution
- This repository includes **Quanser-provided materials**; use is subject to **Quanser’s educational licensing**. Check your lab’s license agreement.
- New code authored by ACL contributors is licensed under this repo’s `LICENSE` unless otherwise stated in a subfolder.

---

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
