# Implementation Plan: Professional Landing Page

**Branch**: `001-landing-page` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-landing-page/spec.md`

## Summary

Create a futuristic landing page for the Physical AI & Humanoid Robotics textbook that serves as the entry point for all users. The page features a 50/50 split layout (desktop) with Three.js animated cube on the left and hero content on the right, followed by 4 glass-morphism stat cards below. Primary goal: enable users to understand the textbook value proposition and navigate to content with one click while maintaining Lighthouse >90 performance and WCAG 2.1 AA accessibility.

**Technical Approach**: Custom Docusaurus page component (`src/pages/index.tsx`) using React 18+, Three.js (lazy-loaded desktop-only), Framer Motion for card animations, Tailwind CSS for styling, and CSS Modules for component-specific styles. Implement responsive breakpoints, reduced-motion detection, and WebGL fallbacks to ensure universal accessibility.

## Technical Context

**Language/Version**: TypeScript 5.x, React 18+, Node.js 18+
**Primary Dependencies**:
- Docusaurus 3.x (core framework)
- Three.js 0.160+ (3D animation)
- @react-three/fiber (React Three.js integration)
- @react-three/drei (Three.js helpers)
- Framer Motion 10+ (UI animations)
- Tailwind CSS 3.x (styling)
- clsx / classnames (conditional classes)

**Storage**: N/A (static page, no data persistence)
**Testing**: Jest + React Testing Library (component tests), Playwright (E2E), Lighthouse CI (performance/accessibility)
**Target Platform**: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+), mobile browsers (iOS Safari, Chrome Android)
**Project Type**: web (Docusaurus static site)
**Performance Goals**:
- Initial load <3s on 3G
- FCP <1.5s
- TTI <3s
- 60fps animations (desktop with discrete GPU)
- Lighthouse Performance >90
- Lighthouse Accessibility >90

**Constraints**:
- Three.js bundle size (~600KB) mitigated by desktop-only lazy loading
- Glass-morphism `backdrop-filter` unsupported in Firefox <103 (fallback: solid semi-transparent BG)
- Docusaurus page routing constraints (must use `src/pages/index.tsx` for root path)

**Scale/Scope**: Single page feature, ~500 LOC (components + styles), 4 React components

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle V: Performance & Accessibility ✅ PASS

- **Requirement**: Landing page load <3s, Lighthouse >90, WCAG 2.1 AA
- **Compliance**:
  - Performance targets defined: FCP <1.5s, TTI <3s, 60fps animations
  - Lazy-load Three.js desktop-only (mobile gets static fallback)
  - Keyboard navigation for button (tabindex, focus indicators)
  - Color contrast ratios: cyan (#00F0FF) on dark BG (#0A0A0F) = 10.4:1 ✅
  - Proper heading hierarchy: h1 for main heading
  - `prefers-reduced-motion` respected (disables animations)
  - Alt text for SVG fallback, ARIA labels for interactive elements

### Principle VI: Brand Consistency ✅ PASS

- **Requirement**: Humaride Robotics color palette and design system
- **Compliance**:
  - Primary cyan: `#00F0FF`
  - Secondary magenta: `#FF2A6D`
  - Dark BG: `#0A0A0F`
  - Card BG (dark): `#12131A`
  - Text (dark): `#D6D6D6`
  - Fonts: Inter Bold 48px (heading), Inter Regular 20px (subheading)
  - Glass-morphism: `backdrop-filter: blur(10px)`
  - Hover effects: `box-shadow: 0 0 20px #00F0FF`

### Additional Checks

- **No educational content on landing page** → Principle I (Educational Quality) not applicable ✅
- **No chapter structure** → Principle II (Content Architecture) not applicable ✅
- **No code examples** → Principle III (Code Correctness) not applicable ✅
- **SVG diagram for fallback** → Principle IV (Visual Learning) partially applicable ✅
- **No auth/personalization on landing page** → Principle VII not applicable (future feature)
- **No chatbot on landing page** → Principle VIII not applicable (future feature)

**Result**: All applicable constitution principles satisfied. No violations to justify.

## Project Structure

### Documentation (this feature)

```text
specs/001-landing-page/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # N/A (no data entities)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # N/A (no API contracts)
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── spec.md              # Feature specification
```

### Source Code (repository root)

```text
src/
├── pages/
│   └── index.tsx                    # Landing page (replaces Docusaurus default)
├── components/
│   ├── LandingPage/
│   │   ├── Hero.tsx                 # Right side: heading, subheading, CTA button
│   │   ├── AnimatedCube.tsx         # Left side: Three.js cube (desktop only)
│   │   ├── StatCard.tsx             # Individual stat card component
│   │   ├── StatsSection.tsx         # 4-card grid with Framer Motion
│   │   ├── Hero.module.css          # Hero component styles
│   │   ├── AnimatedCube.module.css  # Cube container styles
│   │   ├── StatCard.module.css      # Card glass-morphism styles
│   │   └── StatsSection.module.css  # Grid layout styles
│   └── common/
│       └── FallbackSVG.tsx          # Static SVG placeholder (mobile/no WebGL)
├── styles/
│   ├── custom.css                   # Global Docusaurus overrides
│   └── theme-colors.css             # Humaride Robotics color variables
└── hooks/
    ├── useReducedMotion.ts          # Detect prefers-reduced-motion
    ├── useMediaQuery.ts             # Responsive breakpoint detection
    └── useWebGLSupport.ts           # Detect WebGL availability

tests/
├── components/
│   ├── Hero.test.tsx                # Hero component unit tests
│   ├── AnimatedCube.test.tsx        # Cube component unit tests
│   ├── StatCard.test.tsx            # Card component unit tests
│   └── StatsSection.test.tsx        # Stats section unit tests
├── e2e/
│   └── landing-page.spec.ts         # Playwright E2E tests
└── lighthouse/
    └── landing-page.config.js       # Lighthouse CI config
```

**Structure Decision**: Web application structure with Docusaurus conventions. Landing page lives in `src/pages/index.tsx` (Docusaurus requirement for root path). Components follow feature-based organization (`LandingPage/` folder) with co-located CSS Modules. Custom hooks centralized in `src/hooks/` for reusability. Tests mirror source structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations identified.** All constitution checks passed. No complexity justification required.
