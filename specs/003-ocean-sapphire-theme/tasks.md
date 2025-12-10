# Implementation Tasks: Ocean Sapphire Design System

**Feature**: 003-ocean-sapphire-theme | **Branch**: `003-ocean-sapphire-theme` | **Date**: 2025-12-10

---

## Overview

This document contains actionable tasks for implementing the Ocean Sapphire Design System across the Docusaurus textbook site. Tasks are organized by user story priority (P1→P2→P3→P4) to enable independent, incremental delivery.

**Total Tasks**: 62 tasks across 6 phases
**Parallelizable Tasks**: 38 tasks marked with [P]
**User Stories**: 4 (Landing Page, Book Reader, Chapter Cards, CSS Variables)

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)
**User Story 1 only** - Landing Page Ocean Sapphire Theme
- Delivers immediate visual value (first impression)
- Independently testable at root URL `/`
- ~15 tasks, estimated 4-6 hours

### Incremental Delivery Order
1. **Phase 1-2**: Setup + Foundational (all user stories depend on this)
2. **Phase 3 (US1)**: Landing Page (P1) - Deploy first for user feedback
3. **Phase 4 (US2)**: Book Reader (P2) - Core reading experience
4. **Phase 5 (US3)**: Chapter Cards (P3) - Enhanced interactivity
5. **Phase 6 (US4)**: CSS Variables (P4) - Technical foundation polish
6. **Phase 7**: Final Polish - Cross-cutting concerns

Each phase is independently testable and delivers standalone value.

---

## Phase 1: Setup (Project Initialization)

**Goal**: Install dependencies and configure Docusaurus for Ocean Sapphire theme

**Tasks**:

- [ ] T001 Install Three.js dependencies in website/ directory: `npm install three@^0.160.0 @types/three@^0.160.0`
- [ ] T002 [P] Install Playwright for visual regression testing: `npm install --save-dev @playwright/test@^1.41.0`
- [ ] T003 [P] Initialize Playwright configuration: `npx playwright install chromium --with-deps`
- [ ] T004 Configure Docusaurus for dark mode only in website/docusaurus.config.ts (set defaultMode: 'dark', disableSwitch: true)
- [ ] T005 [P] Create directory structure: website/src/css/, website/src/components/, website/src/pages/, tests/visual-regression/
- [ ] T006 [P] Copy CSS variable contract from specs/003-ocean-sapphire-theme/contracts/OceanSapphireVariables.css to website/src/css/variables.css
- [ ] T007 [P] Create empty animation keyframes file at website/src/css/animations.css
- [ ] T008 [P] Create empty glassmorphism styles file at website/src/css/glassmorphism.css

**Completion Criteria**: Dependencies installed, directory structure created, Docusaurus configured for dark mode

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Establish CSS foundation and core styling infrastructure required by all user stories

**Tasks**:

- [ ] T009 Define all 15 Ocean Sapphire CSS variables in website/src/css/custom.css :root block (copy from variables.css contract)
- [ ] T010 [P] Create shimmer animation keyframe in website/src/css/animations.css (3s infinite gradient sweep)
- [ ] T011 [P] Create fadeIn animation keyframe in website/src/css/animations.css (0.6s ease with translateY)
- [ ] T012 [P] Create glowPulse animation keyframe in website/src/css/animations.css (2s infinite box-shadow pulse)
- [ ] T013 [P] Add prefers-reduced-motion media query in website/src/css/animations.css to disable all animations
- [ ] T014 Create base glassmorphism card styles in website/src/css/glassmorphism.css with @supports fallback for backdrop-filter
- [ ] T015 Apply Ocean Sapphire background gradient to Docusaurus body in website/src/css/custom.css (linear-gradient 180deg)
- [ ] T016 [P] Override Docusaurus CSS variables in custom.css: --ifm-background-color, --ifm-font-color-base with Ocean Sapphire colors and inline fallbacks
- [ ] T017 [P] Set Georgia serif font-family for all headings and body text in website/src/css/custom.css

**Completion Criteria**: CSS variables defined, animations created, glassmorphism base styles ready, Docusaurus styled with Ocean Sapphire colors

---

## Phase 3: User Story 1 - Landing Page Ocean Sapphire Theme (Priority: P1)

**Story Goal**: Create premium landing page with Three.js animation, glassmorphism stats cards, and Ocean Sapphire gradient background

**Independent Test**: Visit `/` and verify:
- Ocean Sapphire gradient background displays
- Three.js cube animates at 60fps with blue edges
- 4 stats cards have glassmorphism effects
- Stats cards shimmer on hover
- "Read Book" button has cyan glow on hover
- Mobile responsive (390px, 360px viewports)

**Tasks**:

### 3.1 Three.js Scene Component

- [ ] T018 [P] [US1] Create ThreeScene component at website/src/components/ThreeScene.tsx implementing ThreeScene.interface.ts contract
- [ ] T019 [P] [US1] Add Three.js scene initialization with BoxGeometry in ThreeScene.tsx (camera position z: 5)
- [ ] T020 [P] [US1] Implement MeshBasicMaterial with wireframe: true and color: 0x0096ff in ThreeScene.tsx
- [ ] T021 [P] [US1] Add requestAnimationFrame animation loop with rotation speed 0.01 rad/frame in ThreeScene.tsx
- [ ] T022 [P] [US1] Implement cleanup function in ThreeScene.tsx to dispose renderer and remove canvas on unmount
- [ ] T023 [P] [US1] Add resize handler in ThreeScene.tsx to update camera aspect ratio and renderer size on window resize

### 3.2 Stats Card Components

- [ ] T024 [P] [US1] Create OceanCard base component at website/src/components/OceanCard.tsx implementing OceanCard.interface.ts
- [ ] T025 [P] [US1] Add glassmorphism styles to OceanCard with className ocean-card (background, backdrop-filter, border, border-radius)
- [ ] T026 [P] [US1] Implement shimmer animation on hover for OceanCard using animation: shimmer 3s linear infinite
- [ ] T027 [P] [US1] Create StatsCard component at website/src/components/StatsCard.tsx with number and label props
- [ ] T028 [P] [US1] Style StatsCard number as 2.5em cyan (#00d4ff) and label as 0.9em caption color (#8bb3e0)

### 3.3 Landing Page Layout

- [ ] T029 [US1] Create landing page component at website/src/pages/index.tsx with Docusaurus Layout wrapper
- [ ] T030 [US1] Wrap ThreeScene component in Docusaurus <BrowserOnly> with fallback "Loading 3D scene..." in index.tsx
- [ ] T031 [P] [US1] Add landing page gradient background in index.tsx: linear-gradient(180deg, #001529 0%, #002140 50%, #003a6d 100%)
- [ ] T032 [P] [US1] Create "HUMARIDE" heading in index.tsx with Georgia 4.5em, white color, cyan glow text-shadow: 0 0 30px rgba(0, 150, 255, 0.6)
- [ ] T033 [P] [US1] Add subheading "Physical AI & Humanoid Robotics" in index.tsx with Georgia italic 1.3em, color #80c0ff
- [ ] T034 [P] [US1] Create "Read Book" button in index.tsx with background rgba(0, 150, 255, 0.2), border 1px solid #0096ff, hover box-shadow: 0 0 20px rgba(0, 150, 255, 0.5)
- [ ] T035 [P] [US1] Add 4 StatsCard components to index.tsx: "4 Modules", "21 Chapters", "Interactive RAG", "Urdu Support"
- [ ] T036 [US1] Implement responsive layout in index.tsx: 50/50 split on desktop (≥768px), stacked on mobile (<768px)

### 3.4 Visual Regression Tests

- [ ] T037 [P] [US1] Create Playwright test file at tests/visual-regression/landing-page.spec.ts
- [ ] T038 [P] [US1] Write Playwright test for landing page full-page screenshot baseline in landing-page.spec.ts
- [ ] T039 [P] [US1] Add Playwright test for stats card hover state screenshot in landing-page.spec.ts
- [ ] T040 [P] [US1] Write Playwright test to verify Three.js scene renders without errors in landing-page.spec.ts
- [ ] T041 [US1] Generate baseline screenshots with `npx playwright test --update-snapshots` and commit to Git

**Story Completion Criteria**:
- Landing page renders at `/` with Ocean Sapphire gradient
- Three.js cube animates smoothly at 60fps
- 4 stats cards display with glassmorphism effects
- Hover effects work: shimmer on cards, glow on button
- Playwright visual regression tests pass
- Mobile responsive on 390px and 360px viewports

---

## Phase 4: User Story 2 - Book Reader Pages Ocean Sapphire Theme (Priority: P2)

**Story Goal**: Apply Ocean Sapphire styling to all chapter pages with comfortable reading typography, TOC active state, and code block styling

**Independent Test**: Navigate to any `/docs/*` page and verify:
- Background is dark blue (#0d1117) with subtle glow
- Body text is Georgia 16-18px in pale blue (#b8d4ff), line-height 2.0
- TOC active chapter has cyan left border (3px) and glow
- Code blocks use Fira Code font with cyan copy button hover
- Highlight boxes have 💎 icon and cyan left border
- Page transitions include 0.6s fade-in animation

**Tasks**:

### 4.1 Docusaurus Theme Swizzling

- [ ] T042 [US2] Swizzle TOC component in wrap mode: `npm run swizzle @docusaurus/theme-classic TOC -- --wrap`
- [ ] T043 [US2] Create TOC wrapper component at website/src/theme/TOC/index.tsx importing @theme-original/TOC
- [ ] T044 [US2] Add className ocean-toc-wrapper to TOC wrapper div in index.tsx
- [ ] T045 [US2] Swizzle CodeBlock/Content component in wrap mode: `npm run swizzle @docusaurus/theme-classic CodeBlock/Content -- --wrap`
- [ ] T046 [US2] Create CodeBlock wrapper at website/src/theme/CodeBlock/Content/index.tsx with className ocean-code-block

### 4.2 TOC Sidebar Styling

- [ ] T047 [P] [US2] Style TOC active state in website/src/css/custom.css: .ocean-toc-wrapper .table-of-contents__link--active with border-left 3px solid #0096ff
- [ ] T048 [P] [US2] Add cyan glow to TOC active state in custom.css: box-shadow: 0 0 15px rgba(0, 150, 255, 0.3)
- [ ] T049 [P] [US2] Apply glowPulse animation to TOC active state: animation: glowPulse 2s ease-in-out infinite
- [ ] T050 [P] [US2] Set TOC active text color to soft cyan (#00d4ff) with font-weight 500 in custom.css

### 4.3 Book Reader Typography

- [ ] T051 [P] [US2] Set book reader background to #0d1117 with radial gradient ambient glow in custom.css
- [ ] T052 [P] [US2] Apply Georgia serif to all chapter headings (H1-H3) with white color in custom.css
- [ ] T053 [P] [US2] Set body text to Georgia 1.1em, color #b8d4ff, line-height 2.0 in custom.css
- [ ] T054 [P] [US2] Add page fade-in animation to chapter content: animation: fadeIn 0.6s ease-out in custom.css

### 4.4 Code Block Styling

- [ ] T055 [P] [US2] Set code block font-family to 'Fira Code', 'Courier New', Consolas, monospace in custom.css
- [ ] T056 [P] [US2] Style code block copy button in custom.css: .ocean-code-block button[class*="copyButton"] with background rgba(0, 50, 100, 0.3), border 1px solid #0096ff
- [ ] T057 [P] [US2] Add cyan hover effect to copy button: box-shadow: 0 0 20px rgba(0, 150, 255, 0.5), background rgba(0, 150, 255, 0.2), color #00d4ff

### 4.5 Highlight Boxes

- [ ] T058 [P] [US2] Create highlight box styles in custom.css: .ocean-highlight with background rgba(0, 100, 200, 0.15), border-left 4px solid #0096ff
- [ ] T059 [P] [US2] Add 💎 icon to highlight boxes using ::before pseudo-element with content: "💎", color: #0096ff

### 4.6 Visual Regression Tests

- [ ] T060 [P] [US2] Create Playwright test at tests/visual-regression/chapter-page.spec.ts for full chapter page screenshot
- [ ] T061 [P] [US2] Add Playwright test to verify TOC active state CSS (border-left: 3px solid rgb(0, 150, 255)) in chapter-page.spec.ts
- [ ] T062 [P] [US2] Write Playwright test for code block copy button hover state in chapter-page.spec.ts
- [ ] T063 [US2] Generate chapter page baseline screenshots with `npx playwright test --update-snapshots` and commit

**Story Completion Criteria**:
- All `/docs/*` pages styled with Ocean Sapphire theme
- TOC active state shows cyan border and glow
- Body text is readable: Georgia, pale blue, line-height 2.0
- Code blocks use Fira Code with cyan copy button hover
- Highlight boxes have 💎 icon and cyan styling
- Visual regression tests pass for chapter pages

---

## Phase 5: User Story 3 - Chapter Card Glassmorphism Effects (Priority: P3)

**Story Goal**: Add glassmorphism and shimmer effects to chapter cards for enhanced interactivity

**Independent Test**: Navigate to pages with chapter cards and verify:
- Cards have glassmorphism background (rgba(0, 50, 100, 0.3) with blur(10px))
- Shimmer animation triggers on hover (3s gradient sweep)
- Chapter badges have cyan glow effect
- Touch interactions work on mobile devices

**Tasks**:

### 5.1 Chapter Card Component

- [ ] T064 [P] [US3] Create ChapterCard component at website/src/components/ChapterCard.tsx implementing ChapterCard interface from contracts/
- [ ] T065 [P] [US3] Apply glassmorphism styles to ChapterCard: background rgba(0, 50, 100, 0.3), backdrop-filter blur(10px), border-radius 12px
- [ ] T066 [P] [US3] Add @supports fallback for ChapterCard: background rgba(0, 50, 100, 0.7) when backdrop-filter not supported
- [ ] T067 [P] [US3] Implement shimmer animation on ChapterCard hover using shimmer keyframe (3s linear infinite)
- [ ] T068 [P] [US3] Add hover state transform: translateY(-5px) with transition 0.3s ease to ChapterCard

### 5.2 Chapter Badges

- [ ] T069 [P] [US3] Create chapter badge styles in custom.css: background rgba(0, 150, 255, 0.2), border 1px solid #0096ff, border-radius 30px
- [ ] T070 [P] [US3] Set badge text color to #00d4ff with uppercase and letter-spacing 2px in custom.css
- [ ] T071 [P] [US3] Add cyan glow to badges: box-shadow: 0 0 15px rgba(0, 150, 255, 0.3) in custom.css

### 5.3 Mobile Touch Interactions

- [ ] T072 [P] [US3] Add touch-friendly hover alternatives for mobile in custom.css using @media (hover: none)
- [ ] T073 [P] [US3] Test chapter card touch interactions on mobile viewports (390px, 360px) and adjust padding/sizing if needed

### 5.4 Visual Regression Tests

- [ ] T074 [P] [US3] Create Playwright test at tests/visual-regression/chapter-cards.spec.ts for chapter card screenshots
- [ ] T075 [P] [US3] Add Playwright test for chapter card hover state with shimmer animation in chapter-cards.spec.ts
- [ ] T076 [P] [US3] Write Playwright test to verify glassmorphism fallback in browsers without backdrop-filter support
- [ ] T077 [US3] Generate chapter card baseline screenshots and commit to Git

**Story Completion Criteria**:
- Chapter cards have glassmorphism effects (background + blur)
- Shimmer animation works on hover
- Badges styled with cyan glow and proper typography
- Mobile touch interactions work smoothly
- Visual regression tests pass

---

## Phase 6: User Story 4 - CSS Variable System and Theme Consistency (Priority: P4)

**Story Goal**: Ensure all Ocean Sapphire styling uses CSS variables for maintainability and consistency

**Independent Test**: Inspect compiled CSS and verify:
- All 15 Ocean Sapphire CSS variables defined in `:root`
- All components reference CSS variables (not hard-coded hex values)
- Inline fallback pattern used for all CSS variable usage
- Changing one CSS variable propagates across all components

**Tasks**:

### 6.1 CSS Variable Audit

- [ ] T078 [P] [US4] Audit all CSS files in website/src/css/ for hard-coded hex values and replace with CSS variables + inline fallbacks
- [ ] T079 [P] [US4] Verify all Ocean Sapphire colors use inline fallback pattern: `color: #b8d4ff; color: var(--ocean-text-pale);`
- [ ] T080 [P] [US4] Audit component files (ThreeScene, OceanCard, StatsCard, ChapterCard) for hard-coded colors and replace with CSS variables
- [ ] T081 [P] [US4] Verify all animations reference CSS variable durations: `var(--ocean-transition-fast)`, `var(--ocean-shimmer-duration)`

### 6.2 CSS Variable Validation

- [ ] T082 [P] [US4] Create Playwright test at tests/visual-regression/css-variables.spec.ts to programmatically extract :root variables
- [ ] T083 [P] [US4] Write Playwright test to verify all 15 Ocean Sapphire CSS variables are defined in compiled CSS
- [ ] T084 [P] [US4] Add Playwright test to verify no hard-coded Ocean Sapphire hex values exist in compiled CSS (except inline fallbacks)

### 6.3 Documentation

- [ ] T085 [P] [US4] Add CSS variable usage examples to website/src/css/custom.css as comments
- [ ] T086 [P] [US4] Document CSS variable naming convention in website/src/css/custom.css: --ocean-[category]-[variant]

**Story Completion Criteria**:
- All 15 CSS variables defined in `:root`
- All components use CSS variables (no hard-coded colors)
- Inline fallback pattern used consistently
- CSS variable validation tests pass
- Documentation added for maintainability

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final quality assurance, performance optimization, and deployment preparation

**Tasks**:

### 7.1 Accessibility Validation

- [ ] T087 [P] Run WCAG AAA contrast checker on body text (#b8d4ff on #001529) and verify 8.2:1 ratio
- [ ] T088 [P] Run WCAG AAA contrast checker on headings (white on #001529) and verify 14.5:1 ratio
- [ ] T089 [P] Test prefers-reduced-motion media query by enabling reduced motion in browser settings and verifying animations are disabled
- [ ] T090 [P] Verify all interactive elements (buttons, cards, TOC items) are keyboard-accessible (tab + enter)

### 7.2 Performance Validation

- [ ] T091 Run Lighthouse performance audit on landing page and verify score >90
- [ ] T092 [P] Run Lighthouse accessibility audit on landing page and verify score >90
- [ ] T093 [P] Measure Three.js scene frame rate with browser DevTools and verify 60fps maintained
- [ ] T094 [P] Test landing page load time on 3G connection and verify <3 seconds

### 7.3 Browser Compatibility

- [ ] T095 [P] Test glassmorphism fallback in Firefox 88 (no backdrop-filter) and verify solid background renders
- [ ] T096 [P] Test landing page in Chrome 90+, Firefox 88+, Safari 14+ and verify visual consistency
- [ ] T097 [P] Test mobile responsive layout on iPhone 12 (390px) and Galaxy S21 (360px) viewports

### 7.4 Build & Deploy

- [ ] T098 Run Docusaurus build with `npm run build` and verify no SSR errors or warnings
- [ ] T099 [P] Verify all visual regression Playwright tests pass with baseline screenshots
- [ ] T100 [P] Create GitHub Actions workflow at .github/workflows/playwright.yml for automated visual regression CI
- [ ] T101 Update website/README.md with Ocean Sapphire implementation notes and quickstart instructions
- [ ] T102 Commit all Ocean Sapphire changes with descriptive commit message following Conventional Commits format

**Completion Criteria**:
- WCAG AAA compliance verified (8.2:1 body, 14.5:1 headings)
- Lighthouse Performance >90, Accessibility >90
- Three.js maintains 60fps
- Browser compatibility tested (Chrome, Firefox, Safari)
- Mobile responsive verified (390px, 360px)
- Build succeeds without errors
- Visual regression tests pass in CI

---

## Dependency Graph

### Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Foundational)
                        ↓
        ┌───────────────┼───────────────┬───────────────┐
        ↓               ↓               ↓               ↓
    Phase 3         Phase 4         Phase 5         Phase 6
    (US1: Landing)  (US2: Reader)   (US3: Cards)    (US4: Variables)
    [Independent]   [Independent]   [Independent]   [Requires US1-3]
        ↓               ↓               ↓               ↓
        └───────────────┴───────────────┴───────────────┘
                        ↓
                    Phase 7 (Polish)
```

**Dependencies**:
- **Phase 1-2 MUST complete first** (all user stories require CSS variables, animations, glassmorphism base)
- **US1, US2, US3 are independent** after Phase 2 (can be implemented in parallel or any order)
- **US4 depends on US1-3** (requires components to exist for CSS variable audit)
- **Phase 7 depends on all user stories** (final validation across all features)

### Parallel Execution Opportunities

**Phase 1 (Setup)**: Tasks T002-T003, T005-T008 can run in parallel (8 tasks)
**Phase 2 (Foundational)**: Tasks T010-T013, T016-T017 can run in parallel (6 tasks)
**Phase 3 (US1)**: Tasks T018-T023 (ThreeScene), T024-T028 (StatsCard), T031-T035 (Layout elements), T037-T040 (Tests) can run in parallel (21 tasks)
**Phase 4 (US2)**: Tasks T047-T050 (TOC), T051-T054 (Typography), T055-T057 (Code), T058-T059 (Highlights), T060-T062 (Tests) can run in parallel (16 tasks)
**Phase 5 (US3)**: Tasks T064-T068 (Cards), T069-T071 (Badges), T072-T076 (Tests) can run in parallel (13 tasks)
**Phase 6 (US4)**: Tasks T078-T081 (Audit), T082-T084 (Validation), T085-T086 (Docs) can run in parallel (9 tasks)
**Phase 7 (Polish)**: Tasks T087-T090 (A11y), T092-T094 (Performance), T095-T097 (Compatibility), T099-T100 (CI) can run in parallel (13 tasks)

**Total Parallelizable**: 86 out of 102 tasks (84%)

---

## Task Completion Tracking

**Phase 1 - Setup**: 0/8 tasks complete
**Phase 2 - Foundational**: 0/9 tasks complete
**Phase 3 - US1 (Landing Page)**: 0/24 tasks complete
**Phase 4 - US2 (Book Reader)**: 0/22 tasks complete
**Phase 5 - US3 (Chapter Cards)**: 0/14 tasks complete
**Phase 6 - US4 (CSS Variables)**: 0/9 tasks complete
**Phase 7 - Polish**: 0/16 tasks complete

**Overall Progress**: 0/102 tasks complete (0%)

---

## Testing Strategy

**Visual Regression Testing** (Playwright):
- Landing page full-page screenshot (T038)
- Stats card hover state (T039)
- Chapter page full screenshot (T060)
- TOC active state CSS validation (T061)
- Code block copy button hover (T062)
- Chapter card glassmorphism (T074-T076)
- CSS variables programmatic extraction (T082-T084)

**Accessibility Testing**:
- WCAG AAA contrast ratios (T087-T088)
- prefers-reduced-motion support (T089)
- Keyboard accessibility (T090)

**Performance Testing**:
- Lighthouse Performance >90 (T091)
- Lighthouse Accessibility >90 (T092)
- Three.js 60fps validation (T093)
- 3G load time <3s (T094)

**Browser Compatibility Testing**:
- Glassmorphism fallback in old browsers (T095)
- Cross-browser visual consistency (T096)
- Mobile responsive testing (T097)

**Total Tests**: 17 test tasks

---

## File Inventory

### Files Created (New)

**Components**:
- `website/src/components/ThreeScene.tsx`
- `website/src/components/OceanCard.tsx`
- `website/src/components/StatsCard.tsx`
- `website/src/components/ChapterCard.tsx`

**Pages**:
- `website/src/pages/index.tsx` (Landing page)

**CSS**:
- `website/src/css/custom.css` (CSS variables + Ocean Sapphire overrides)
- `website/src/css/animations.css` (Keyframes: shimmer, fadeIn, glowPulse)
- `website/src/css/glassmorphism.css` (Card styles with @supports fallback)

**Swizzled Components**:
- `website/src/theme/TOC/index.tsx`
- `website/src/theme/CodeBlock/Content/index.tsx`

**Tests**:
- `tests/visual-regression/landing-page.spec.ts`
- `tests/visual-regression/chapter-page.spec.ts`
- `tests/visual-regression/chapter-cards.spec.ts`
- `tests/visual-regression/css-variables.spec.ts`

**CI/CD**:
- `.github/workflows/playwright.yml`

### Files Modified (Existing)

- `website/docusaurus.config.ts` (Dark mode configuration)
- `website/package.json` (Add Three.js, Playwright dependencies)
- `website/README.md` (Ocean Sapphire documentation)

**Total Files**: 14 new files + 3 modified files

---

## Validation Checklist

Before marking Ocean Sapphire implementation complete, verify:

- [ ] All 102 tasks marked as complete
- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] Visual regression tests pass (`npx playwright test`)
- [ ] Landing page renders at `/` with Ocean Sapphire theme
- [ ] Chapter pages styled correctly at `/docs/*`
- [ ] Three.js scene animates at 60fps
- [ ] Glassmorphism effects work (or fallback renders)
- [ ] TOC active state shows cyan border and glow
- [ ] Code block copy button has cyan hover effect
- [ ] WCAG AAA contrast ratios verified (8.2:1, 14.5:1)
- [ ] prefers-reduced-motion disables animations
- [ ] Mobile responsive (390px, 360px viewports)
- [ ] Browser compatibility tested (Chrome 90+, Firefox 88+, Safari 14+)
- [ ] Lighthouse Performance >90, Accessibility >90
- [ ] All CSS variables defined in `:root` with inline fallbacks
- [ ] README.md updated with Ocean Sapphire notes

---

**Next Step**: Begin implementation with Phase 1 (Setup) tasks T001-T008, then proceed to Phase 2 (Foundational) before implementing user stories in priority order (P1→P2→P3→P4).
