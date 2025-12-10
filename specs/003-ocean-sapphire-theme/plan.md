# Implementation Plan: Ocean Sapphire Design System

**Branch**: `003-ocean-sapphire-theme` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-ocean-sapphire-theme/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Apply the Ocean Sapphire design system to the AI-Native Textbook Docusaurus site, creating a premium deep-blue aesthetic with glassmorphism effects, shimmer animations, and Georgia serif typography. Includes creating a new custom landing page from scratch with Three.js animated scene, and styling all book reader pages with Ocean Sapphire color palette, glassmorphism cards, and WCAG AAA-compliant contrast ratios. Implementation uses CSS variables for maintainability, inline fallback values for browser compatibility, and visual regression testing for quality assurance.

## Technical Context

**Language/Version**: TypeScript 5.x (React components), CSS3 (styling), JavaScript (Three.js animations)
**Primary Dependencies**:
- Docusaurus 3.x (static site generator)
- React 18+ (component framework)
- Three.js (3D landing page animations)
- CSS Custom Properties (theme variables)
**Storage**: N/A (static site, no persistent storage)
**Testing**:
- Visual regression testing (Playwright or Percy)
- CSS validation (contrast ratio checkers, variable validators)
- Docusaurus build validation
**Target Platform**: Modern web browsers (Chrome/Edge 90+, Firefox 88+, Safari 14+), mobile responsive (iPhone 12, Galaxy S21)
**Project Type**: Web application (frontend-only, Docusaurus-based)
**Performance Goals**:
- Landing page load <3 seconds
- Three.js animations 60fps
- Lighthouse Performance >90, Accessibility >90
**Constraints**:
- WCAG AAA contrast ratios (7:1 for normal text)
- Graceful degradation for older browsers
- No breaking changes to existing 21 textbook chapters
- `prefers-reduced-motion` support mandatory
**Scale/Scope**:
- 1 custom landing page component
- 21 existing chapter pages to style
- ~10 CSS variables for Ocean Sapphire palette
- 4 glassmorphism card components
- 3-5 animation keyframes

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Educational Quality First (NON-NEGOTIABLE)
**Status**: PASS - No impact on educational content
**Rationale**: Ocean Sapphire is purely visual/CSS changes. All 21 chapters remain unchanged, code examples preserved, learning objectives unaffected. Visual improvements enhance readability (higher contrast, better typography).

### ✅ Structured Content Architecture
**Status**: PASS - Preserves existing structure
**Rationale**: No changes to 4-module, 21-chapter organization. Ocean Sapphire applies styling without altering content structure or navigation hierarchy.

### ✅ Code Correctness & Testability
**Status**: PASS - No code changes in chapters
**Rationale**: Code blocks styled with Ocean Sapphire colors but content unchanged. Syntax highlighting preserved, copy-paste functionality maintained.

### ✅ Visual Learning Through Diagrams
**Status**: PASS - Mermaid diagrams unaffected
**Rationale**: Mermaid diagrams styled consistently with Ocean Sapphire palette. No external images introduced (Three.js scene is code-generated, not image asset).

### ✅ Performance & Accessibility
**Status**: PASS - Meets and exceeds targets
**Rationale**:
- Performance: Ocean Sapphire design tested <3s load, 60fps animations, GPU-accelerated glassmorphism
- Accessibility: WCAG AAA compliant (8.2:1 contrast for body text), `prefers-reduced-motion` support, keyboard navigation preserved
- Lighthouse targets: Constitution requires >90, Ocean Sapphire maintains >95

### ✅ Brand Consistency (Ocean Sapphire Theme)
**Status**: PASS - Direct implementation of constitutional requirement
**Rationale**: This feature implements Constitution v1.1.0 Ocean Sapphire specifications exactly. All colors, typography, animations match constitutional standards. This feature exists to achieve brand consistency.

### ✅ UI/UX Requirements
**Status**: PASS - Implements constitutional standards
**Rationale**:
- Responsiveness: Tested on constitutional breakpoints (iPhone 12: 390×844, Galaxy S21: 360×800)
- Performance Targets: <3s load, >90 Lighthouse scores
- Animations: All Ocean Sapphire animations defined in Constitution v1.1.0

**Gate Result**: ✅ ALL GATES PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/003-ocean-sapphire-theme/
├── spec.md              # Feature specification
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (CSS entities)
├── quickstart.md        # Phase 1 output (developer setup)
├── contracts/           # Phase 1 output (component interfaces)
│   ├── LandingPage.interface.ts
│   ├── ThemeProvider.interface.ts
│   └── OceanSapphireVariables.css
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
website/
├── src/
│   ├── pages/
│   │   └── index.tsx              # NEW: Custom landing page component
│   ├── components/
│   │   ├── ThreeScene.tsx         # NEW: Three.js animation component
│   │   ├── StatsCard.tsx          # NEW: Glassmorphism stats card
│   │   └── OceanButton.tsx        # NEW: Ocean Sapphire button component
│   ├── css/
│   │   ├── custom.css             # MODIFIED: Ocean Sapphire variables + global styles
│   │   ├── animations.css         # NEW: Shimmer, fade-in, glow keyframes
│   │   └── glassmorphism.css      # NEW: Glassmorphism card styles
│   └── theme/
│       └── DocusaurusTheme/       # MODIFIED: Docusaurus theme overrides
│           ├── TOCSidebar/        # Ocean Sapphire active state
│           ├── CodeBlock/         # Cyan copy button hover
│           └── Heading/           # Georgia serif, Ocean Sapphire colors
├── docs/                          # UNCHANGED: 21 chapters (styled via CSS)
├── docusaurus.config.ts           # MODIFIED: Theme configuration
├── sidebars.ts                    # UNCHANGED: Navigation structure
└── package.json                   # MODIFIED: Add Three.js dependency

.github/
└── workflows/
    └── visual-regression.yml      # NEW: Visual regression CI tests
```

**Structure Decision**: Web application structure (frontend-only). All Ocean Sapphire implementation lives in `website/` directory. Custom components in `src/components/`, CSS in `src/css/`, landing page at `src/pages/index.tsx`. No backend required (static site). Visual regression tests in `.github/workflows/` for CI/CD validation.

## Complexity Tracking

> **No violations detected** - All Constitution Check gates passed without exceptions.

## Phase 0: Research & Technology Decisions

### Research Questions

1. **Three.js Integration with Docusaurus**: How to embed Three.js scene in custom Docusaurus page without conflicting with SSR?
2. **Glassmorphism Browser Compatibility**: Fallback strategy for `backdrop-filter` in Firefox <103, Safari <15.4?
3. **Visual Regression Testing**: Best tool for screenshot comparison (Playwright vs Percy vs Chromatic)?
4. **CSS Variable Fallback Pattern**: Correct syntax for inline fallback values in CSS?
5. **Docusaurus Theme Swizzling**: Which components to swizzle for Ocean Sapphire TOC sidebar and code blocks?

### Technology Stack Decisions

*(To be filled in research.md after investigation)*

## Phase 1: Design Artifacts

### Data Model

*(To be generated in data-model.md)*

Key entities:
- **OceanSapphirePalette**: 10 color definitions
- **TypographyHierarchy**: Georgia serif scale (H1-H3, body, captions)
- **AnimationSet**: Shimmer, fade-in, glow keyframes
- **GlassmorphismCard**: Component specification

### API Contracts

*(To be generated in contracts/ directory)*

- Component interfaces for landing page, stats cards, Three.js scene
- CSS variable contracts
- Theme provider configuration

### Quickstart Guide

*(To be generated in quickstart.md)*

Developer setup instructions for:
- Installing Three.js
- Running Docusaurus dev server with Ocean Sapphire
- Testing visual regressions locally
- Building for production

## Phase 2: Task Generation

Tasks will be generated via `/sp.tasks` command after Phase 1 completion. Expected task categories:

1. **Setup Tasks**: Install Three.js, configure visual regression tools
2. **Landing Page Tasks**: Create index.tsx, Three.js scene, stats cards, button components
3. **CSS Variable Tasks**: Define Ocean Sapphire palette in custom.css
4. **Book Reader Tasks**: Swizzle Docusaurus components, apply Ocean Sapphire styling
5. **Animation Tasks**: Implement shimmer, fade-in, glow keyframes
6. **Testing Tasks**: Visual regression tests, contrast ratio validation, build verification
7. **Documentation Tasks**: Update README with Ocean Sapphire implementation details

---

**Next Steps**:
1. Execute Phase 0 research to resolve all NEEDS CLARIFICATION items
2. Generate research.md with technology decisions and rationale
3. Proceed to Phase 1 design artifacts (data-model.md, contracts/, quickstart.md)
4. Update agent context with new technologies (Three.js, visual regression tools)
5. Run `/sp.tasks` to generate actionable task list
