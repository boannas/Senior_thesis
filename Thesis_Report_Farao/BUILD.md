# Building and previewing `Thesis_Report_Farao`

This project uses **XeLaTeX** (required for `fontspec` and Thai fonts in `config.tex`).

## 1. Install LaTeX (once)

**Ubuntu / Debian:**
```bash
sudo apt update
sudo apt install -y texlive-xetex texlive-latex-extra texlive-fonts-recommended latexmk
```

**macOS (Homebrew):**
```bash
brew install --cask mactex
```

## 2. Compile from terminal

```bash
cd /home/fibo1/foo_thesis/Senior_thesis/Thesis_Report_Farao
xelatex -interaction=nonstopmode thesis.tex
xelatex -interaction=nonstopmode thesis.tex   # run twice for TOC/refs
```

Or with `latexmk`:
```bash
latexmk -xelatex -interaction=nonstopmode thesis.tex
```

Output: `thesis.pdf` in this folder.

## 3. Preview in Cursor / VS Code (recommended)

1. Install extension: **LaTeX Workshop** (`James-Yu.latex-workshop`).
2. Open folder `Thesis_Report_Farao` (or the whole `Senior_thesis` repo).
3. Open `thesis.tex`.
4. Set recipe to XeLaTeX: in Settings, search `latex-workshop.latex.tools` or use the provided `.vscode/settings.json`.
5. **Build:** `Ctrl+Alt+B` (or Command Palette → “LaTeX Workshop: Build LaTeX project”).
6. **PDF preview:** `Ctrl+Alt+V` (or click the preview icon in the editor tab).

PDF updates after each successful build (forward search from `.tex` to PDF is supported).

## 4. Overleaf

Upload the entire `Thesis_Report_Farao` folder (including `font/` and `chapters/`). Set compiler to **XeLaTeX** in Menu → Settings → Compiler.

## 5. PDF preview not showing in Cursor?

**LaTeX Workshop only opens the PDF after a successful build.** If build fails, there is no PDF to preview.

### Checklist

1. **Open the correct folder**  
   Open `Senior_thesis` (or `Thesis_Report_Farao`) in Cursor. Root file must be `Thesis_Report_Farao/thesis.tex`.

2. **Install missing package (most common on Ubuntu)**  
   ```bash
   sudo apt install texlive-science
   ```
   Error in log: `File 'algorithm.sty' not found` → this fixes it.

3. **Build manually and read errors**  
   - Command Palette (`Ctrl+Shift+P`) → **LaTeX Workshop: Build LaTeX project**  
   - Or open **Output** panel → dropdown **LaTeX Workshop** → read red errors.

4. **Open PDF after build**  
   - Command Palette → **LaTeX Workshop: View LaTeX PDF**  
   - Or shortcut: `Ctrl+Alt+V`  
   - PDF appears in a **tab** next to your `.tex` (not a separate browser unless configured).

5. **Confirm PDF exists on disk**  
   ```bash
   ls Thesis_Report_Farao/thesis.pdf
   ```
   If missing, build did not succeed.

6. **Reload window** after installing TeX packages: `Ctrl+Shift+P` → **Developer: Reload Window**.

## 6. Common issues

| Problem | Fix |
|--------|-----|
| `front/title` not found | Use `\include{front/Title}` (Linux is case-sensitive). |
| `KMUTT_logo.png` missing | Optional; placeholder text is used if file absent. |
| `THSarabunNew.ttf` not found | Keep fonts in `font/` subfolder as configured in `config.tex`. |
| pdfLaTeX errors on `fontspec` | Switch compiler to **XeLaTeX**, not pdfLaTeX. |
| No preview / blank | Build failed — install `texlive-science`, rebuild, then View PDF. |
| PDF text looks empty / boxes / `[?]` everywhere | **Font:** Linux lacks Times New Roman (fixed in `config.tex` with TeX Gyre Termes fallback). **Citations:** enable `\include{back/biography}` at end of `thesis.tex`. |
| Wrong compiler | Must use **XeLaTeX** (magic comment `% !TEX program = xelatex` in `thesis.tex`). |

## 6. Project structure

```
Thesis_Report_Farao/
  thesis.tex          ← main file (compile this)
  config.tex
  chapters/           ← 1_introduction … 5_conclusion
  front/              ← Title, Abstract, …
  diagram/            ← TikZ inputs
  font/               ← Thai fonts
  BUILD.md            ← this file
```
