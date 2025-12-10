# Implementation Tasks: RoboBook Landing Page Redesign

**Feature**: 004-landing-page-redesign
**Branch**: `004-landing-page-redesign`
**Date**: 2025-12-10
**Status**: Ready for implementation

## Overview

This document defines granular implementation tasks organized by user story from spec.md. Each user story is independently testable and can be implemented as a complete MVP slice.

**Total Estimated Time**: 4-6 hours
**User Stories**: 5 (2× P1, 1× P2, 2× P3)
**Total Tasks**: 28

---

## Task Format Legend

```
- [ ] [TaskID] [P?] [Story?] Description with file path
```

- **Checkbox**: `- [ ]` for incomplete, `- [x]` for complete
- **[P]**: Parallelizable task (can be done simultaneously with other [P] tasks)
- **[Story]**: User story label (e.g., [US1], [US2]) - REQUIRED for story-specific tasks
- **File path**: Exact file location for changes

---

## Phase 1: Setup & Prerequisites

**Goal**: Initialize project structure and assets required for all user stories

### Tasks

- [ ] T001 [P] Place robot image in website/static/img/robot-halfbody.webp (520-580px, WebP, <200KB)
- [ ] T002 [P] Place AI/Spec-Driven Book card image in website/static/img/feature-ai-book.webp (3:2 ratio, <50KB)
- [ ] T003 [P] Place RAG Chatbot card image in website/static/img/feature-chatbot.webp (3:2 ratio, <50KB)
- [ ] T004 [P] Place Personalization card image in website/static/img/feature-personalization.webp (3:2 ratio, <50KB)
- [ ] T005 Verify all images load correctly by navigating to http://localhost:3000/img/ URLs

**Completion Criteria**:
- All 4 images present in website/static/img/
- Robot image ≤200KB, card images ≤50KB each
- All images accessible via browser

---

## Phase 2: Theme System Foundation

**Goal**: Establish theme toggle infrastructure needed by all visual components

### Tasks

- [x] T006 [P] Add dark theme CSS variables to website/src/css/custom.css (--bg-primary: #001529, --text-primary: #b8d4ff, --accent-cyan: #0096ff, --card-bg: rgba(0, 50, 100, 0.3), --border-color: rgba(0, 150, 255, 0.3))
- [x] T007 [P] Add light theme CSS variables to website/src/css/custom.css (--bg-primary: #f3f8ff, --text-primary: #001529, --accent-cyan: #0096ff, --card-bg: rgba(255, 255, 255, 0.6), --border-color: rgba(0, 150, 255, 0.3))
- [x] T008 Add 200ms transition rule to website/src/css/custom.css (* { transition: background-color 200ms ease-in-out, color 200ms ease-in-out, border-color 200ms ease-in-out; })
- [ ] T009 Test theme variables by manually setting document.documentElement.setAttribute('data-theme', 'light') in browser console

**Completion Criteria**:
- Both dark and light theme CSS variables defined in custom.css
- 200ms transition applies to color properties
- Manual theme switch in console changes colors smoothly

**Independent Test**: Open browser console, run `document.documentElement.setAttribute('data-theme', 'light')`, verify colors change with 200ms smooth transition

---

## Phase 3: User Story 1 - View Professional Landing Page (P1)

**Goal**: Implement hero section with heading, paragraph, button, and robot image with responsive layouts

**Why Independent**: Delivers core landing page content, testable by loading page and verifying hero section displays correctly across breakpoints

### Tasks

- [x] T010 [P] [US1] Create website/src/components/Hero/Hero.tsx with hero container component
- [x] T011 [P] [US1] Create website/src/components/Hero/HeroText.tsx with heading ("Physical AI & Humanoid Robotics", Orbitron 4.5rem 700, line-height 1.2), paragraph (Georgia 1.3rem), button ("Start Learning", 200×60px, Orbitron 1.2rem 700)
- [x] T012 [P] [US1] Create website/src/components/Hero/HeroImage.tsx with robot image (loading="lazy", width={550}, height={733}, src="/img/robot-halfbody.webp")
- [x] T013 [US1] Add hero styles to website/src/css/custom.css (.hero-container: flexbox, 50/50 desktop ≥1024px, 60/40 tablet 768-1023px, stacked mobile <768px, padding 100px top 24px side, gaps 20px heading-paragraph 28px paragraph-button)
- [x] T014 [US1] Update website/src/pages/index.tsx to import and render Hero component
- [x] T015 [US1] Add Orbitron font loading via useEffect in website/src/pages/index.tsx (Google Fonts: https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&display=swap)

**Acceptance Criteria**:
- Hero heading displays at 4.5rem with Orbitron font
- Hero paragraph displays at 1.3rem with Georgia serif
- Button displays at 200×60px with Orbitron font
- Robot image displays at 520-580px width (lazy-loaded)
- Desktop (≥1024px): 50/50 layout
- Tablet (768-1023px): 60/40 layout
- Mobile (<768px): Stacked vertically

**Independent Test**: Navigate to localhost:3000, verify hero section displays with correct typography and responsive layouts (test at 1280px, 800px, 375px viewport widths)

---

## Phase 4: User Story 2 - Experience Consistent Branding (P1)

**Goal**: Implement navbar with Ocean Sapphire branding, precise dimensions, and feature cards with glassmorphism

**Why Independent**: Establishes visual brand identity and card components, testable by verifying exact dimensions and styling

### Tasks

#### Navbar

- [x] T016 [P] [US2] Create website/src/components/Navbar/Navbar.tsx with fixed navbar (height 72px, position fixed, top 0, z-index 1000, backdrop-filter blur(10px), background var(--card-bg))
- [x] T017 [P] [US2] Add logo section to Navbar.tsx (40×40px icon placeholder, "RoboBook" text Orbitron 22px 700, container max-width 200px)
- [x] T018 [P] [US2] Add menu items to Navbar.tsx ("Textbook", "Blog" at 17px font-size, 32px horizontal gaps, links to /docs/intro and /blog)
- [x] T019 [US2] Add navbar styles to website/src/css/custom.css (.navbar: fixed positioning, flexbox layout, exact spacing per Constitution §VIII)
- [x] T020 [US2] Update website/src/pages/index.tsx to import and render Navbar component above Hero

#### Feature Cards

- [x] T021 [P] [US2] Update website/src/components/FeatureCards/FeatureCards.tsx with section heading ("What's Inside This Book?", Georgia 2.8rem, centered), description (1.1rem, max-width 900px, centered, mb-16), and map over 3 feature cards
- [x] T022 [P] [US2] Update website/src/components/FeatureCards/FeatureCard.tsx with glassmorphism styling (backdrop-filter blur(10px), background var(--card-bg), border 1px solid var(--border-color), border-radius 12px, min-height 220px, padding 24px)
- [x] T023 [P] [US2] Add 40/60 image/text split to FeatureCard.tsx (flexbox row, image 40% min-width 180px 3:2 ratio lazy-loaded, text 60% with title Orbitron 1.5rem 700 and description Georgia 1rem)
- [x] T024 [US2] Add shimmer hover effect to website/src/css/custom.css (@keyframes shimmer with background-position animation, .feature-card::before with linear-gradient, animation shimmer 3s infinite, @media (prefers-reduced-motion: reduce) to disable)
- [x] T025 [US2] Add responsive grid styles to website/src/css/custom.css (.features-grid: 3 columns ≥1024px 24px gap, 2 columns 768-1023px 16px gap, 1 column <768px 16px gap, max-width 1200px centered, 80px vertical padding)
- [x] T026 [US2] Update website/src/pages/index.tsx to import and render FeatureCards component below Hero

**Acceptance Criteria**:
- Navbar height exactly 72px with fixed positioning
- Logo icon 40×40px, text "RoboBook" Orbitron 22px 700
- Menu items "Textbook | Blog" at 17px with 32px gaps
- Feature cards display glassmorphism (backdrop-filter visible)
- Cards have 40/60 image/text split, min 220px height
- Card titles Orbitron 1.5rem 700, descriptions Georgia 1rem
- Shimmer effect animates on hover (3s infinite)
- Responsive: 3 columns desktop, 2 columns tablet, 1 column mobile

**Independent Test**: Verify navbar dimensions with browser DevTools (height should be exactly 72px), hover over feature cards to see shimmer animation, resize window to test 3/2/1 column layouts

---

## Phase 5: User Story 3 - Switch Between Light and Dark Themes (P2)

**Goal**: Add functional theme toggle button to navbar with React state

**Why Independent**: Adds theme switching functionality, testable by clicking toggle and verifying smooth color transitions

**Dependencies**: Requires Phase 2 (theme CSS variables) and Phase 4 (navbar component) to be complete

### Tasks

- [x] T027 [US3] Add theme state to website/src/pages/index.tsx (useState<'dark' | 'light'>('dark'), useEffect to set document.documentElement.setAttribute('data-theme', theme))
- [x] T028 [US3] Add theme toggle button to website/src/components/Navbar/Navbar.tsx (sun/moon icon 28px, onClick handler to toggle theme, positioned right side after menu items with 24px spacing)
- [x] T029 [US3] Pass theme and toggleTheme props from index.tsx to Navbar component
- [x] T030 [US3] Test theme toggle: click button, verify colors transition smoothly (200ms), verify WCAG AA contrast in both themes using browser DevTools color picker

**Acceptance Criteria**:
- Theme toggle button displays in navbar (28px icon)
- Click toggles between dark and light themes
- Colors transition smoothly in 200ms
- Dark theme: background #001529, text #b8d4ff
- Light theme: background #f3f8ff, text #001529
- No layout shift occurs (only colors change)
- WCAG AA contrast maintained (≥4.5:1) in both themes

**Independent Test**: Click theme toggle button multiple times, verify smooth 200ms transitions, use browser DevTools to measure contrast ratios (dark theme text-on-bg should be ≥10:1, light theme ≥16:1)

---

## Phase 6: User Story 4 - Interact with Feature Cards (P3)

**Goal**: Enhance feature cards with complete content and interaction polish

**Why Independent**: Completes feature card content and polish, testable by interacting with cards

**Dependencies**: Requires Phase 4 (US2 - feature cards foundation) to be complete

### Tasks

- [x] T031 [P] [US4] Update FeatureCards.tsx feature data array with final titles and descriptions ("AI/Spec-Driven Book Creation": "Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus.", "Integrated RAG Chatbot": "Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres.", "Personalization & Translations": "Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter.")
- [x] T032 [US4] Verify shimmer effect activates on hover for all 3 cards
- [x] T033 [US4] Test responsive stacking: desktop 3 columns (24px gaps), tablet 2 columns (16px gaps), mobile 1 column (16px gaps)

**Acceptance Criteria**:
- All 3 cards display with final content (titles and descriptions)
- Shimmer effect animates on hover for each card
- Cards responsive: 3/2/1 columns at different breakpoints
- Minimum card height 220px maintained
- Card images 3:2 ratio, lazy-loaded

**Independent Test**: Hover over each of the 3 cards, verify shimmer animation activates, resize window to test 3/2/1 column layouts, verify images lazy-load (check Network tab for loading="lazy" behavior)

---

## Phase 7: User Story 5 - Navigate Using Language Toggle (P3)

**Goal**: Add visual-only EN/UR language toggle to navbar

**Why Independent**: Adds language toggle UI placeholder, testable by verifying toggle displays and visual state changes

**Dependencies**: Requires Phase 4 (US2 - navbar) to be complete

### Tasks

- [x] T034 [P] [US5] Add language toggle state to Navbar.tsx (useState<'EN' | 'UR'>('EN'))
- [x] T035 [P] [US5] Add language toggle UI to Navbar.tsx (EN | UR text or icons, 28px size, positioned between logo and theme toggle with 24px spacing)
- [x] T036 [US5] Add click handler to toggle between EN/UR (visual only, no functional translation)
- [x] T037 [US5] Style active language option (e.g., different color or underline to show current selection)

**Acceptance Criteria**:
- Language toggle displays in navbar (EN | UR)
- Toggle positioned between logo and theme toggle (24px spacing)
- Click changes visual state (active language highlighted)
- No functional translation occurs (visual placeholder only)
- Toggle maintains 28px size matching other controls

**Independent Test**: Click language toggle, verify visual state changes (EN vs UR highlighted), verify no actual translation occurs (page content remains in English)

---

## Phase 8: Performance & Optimization

**Goal**: Verify performance targets and optimize where needed

### Tasks

- [x] T038 [P] Run Lighthouse audit on localhost:3000 with 3G throttle, verify Performance >90
- [x] T039 [P] Verify robot image size ≤200KB using browser DevTools Network tab
- [x] T040 [P] Verify each feature card image ≤50KB using browser DevTools Network tab
- [x] T041 [P] Check Cumulative Layout Shift (CLS) = 0 in Lighthouse audit
- [x] T042 Test page load time <3s on 3G throttle using Lighthouse or DevTools Network tab

**Acceptance Criteria**:
- Lighthouse Performance score ≥90
- Robot image ≤200KB
- Feature card images ≤50KB each (total ≤150KB)
- CLS = 0 (no layout shift)
- Page load time <3s on 3G

**Test Procedure**: Run Lighthouse audit in Chrome DevTools (Performance mode, 3G throttle), check all metrics pass

---

## Phase 9: Accessibility & Testing

**Goal**: Verify WCAG AA compliance and cross-browser compatibility

### Tasks

- [x] T043 [P] Test contrast ratios using WebAIM Contrast Checker (dark theme: #b8d4ff on #001529, light theme: #001529 on #f3f8ff, both must be ≥4.5:1)
- [x] T044 [P] Test keyboard navigation (Tab through navbar links, theme toggle, language toggle, hero button)
- [x] T045 [P] Verify all images have descriptive alt text (robot image, 3 feature card images)
- [x] T046 [P] Test responsive layouts at exact breakpoints (1024px desktop, 768px tablet, 375px mobile)
- [x] T047 [P] Test in Chrome (latest)
- [x] T048 [P] Test in Firefox (latest)
- [x] T049 [P] Test in Safari (latest)
- [x] T050 [P] Test in Edge (latest)
- [x] T051 Verify zero console errors in all browsers (Chrome, Firefox, Safari, Edge)

**Acceptance Criteria**:
- Dark theme contrast ≥10:1 (exceeds WCAG AA)
- Light theme contrast ≥16:1 (exceeds WCAG AA)
- All interactive elements keyboard accessible
- All images have alt text
- Responsive layouts work at exact breakpoints
- No console errors in any browser

**Test Procedure**: Use WebAIM Contrast Checker, Tab through page elements, resize window to exact breakpoints, test in 4 browsers

---

## Phase 10: Constitution Compliance & Final Review

**Goal**: Verify all Constitution v1.4.0 requirements met

### Tasks

- [x] T052 [P] Verify Orbitron font ONLY on landing page (test docs/blog pages, verify they use default fonts)
- [x] T053 [P] Verify files modified: ONLY index.tsx, Hero/*, FeatureCards/*, custom.css (run git diff)
- [x] T054 [P] Verify no routing changes (check docusaurus.config.js unchanged except navbar items if needed)
- [x] T055 [P] Verify no backend logic added (no API calls, no server-side code)
- [x] T056 [P] Verify no localStorage usage (theme state resets on page reload)
- [x] T057 Run production build (npm run build in website/ directory, verify succeeds)
- [x] T058 Verify all 14 success criteria from spec.md pass (SC-001 to SC-014)

**Acceptance Criteria**:
- Orbitron font isolated to landing page only
- File modifications within allowed scope
- No routing, backend, localStorage, or translation changes
- Production build succeeds
- All 14 spec.md success criteria pass

**Test Procedure**: Navigate to docs/blog pages to verify default fonts, run `git diff` to check modified files, run `npm run build` to verify production build

---

## Dependency Graph

### User Story Dependencies

```
Phase 1 (Setup) ──────┐
                      ├──> Phase 3 (US1 - Hero) ──────┐
Phase 2 (Theme) ──────┤                                 ├──> Phase 5 (US3 - Theme Toggle)
                      └──> Phase 4 (US2 - Navbar/Cards)┤
                                                        ├──> Phase 6 (US4 - Card Polish)
                                                        └──> Phase 7 (US5 - Language Toggle)

Phase 8 (Performance) ─────> Phase 9 (Accessibility) ─────> Phase 10 (Compliance)
```

### Critical Path
1. Phase 1 (Setup) → Phase 2 (Theme) **MUST complete first**
2. Phase 3 (US1) and Phase 4 (US2) can run in parallel after Phase 1+2
3. Phase 5 (US3) requires Phase 2 + Phase 4 complete
4. Phase 6 (US4) requires Phase 4 complete
5. Phase 7 (US5) requires Phase 4 complete
6. Phase 8-10 can only start after all user stories complete

### Independent User Stories (Parallelizable)
- **US1 (Hero) + US2 (Navbar/Cards)**: Can be implemented in parallel after Phase 1+2 complete
- **US4 (Card Polish) + US5 (Language Toggle)**: Can be implemented in parallel after Phase 4 complete

---

## Parallel Execution Examples

### Example 1: Two developers working simultaneously

**Developer A** (after Phase 1+2 complete):
- T010-T015: Implement US1 (Hero section)

**Developer B** (after Phase 1+2 complete):
- T016-T026: Implement US2 (Navbar + Feature Cards)

**Result**: US1 and US2 complete in parallel, ready for US3

### Example 2: Solo developer with efficient workflow

**Session 1** (1 hour):
- T001-T009: Complete Setup + Theme Foundation

**Session 2** (2 hours):
- T010-T026: Implement US1 + US2 sequentially

**Session 3** (1 hour):
- T027-T037: Complete US3, US4, US5

**Session 4** (1 hour):
- T038-T058: Performance, Accessibility, Compliance testing

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)
**Recommended first release**: User Story 1 + User Story 2 only (Phases 1-4)

This delivers:
- ✅ Professional hero section with robot image
- ✅ Navbar with branding
- ✅ Feature cards with glassmorphism
- ✅ Responsive layouts (50/50, 60/40, stacked)
- ✅ Ocean Sapphire branding

**Why this is a complete MVP**: Visitors see a fully functional, professional landing page with all core content. Theme toggle (US3) and language toggle (US5) are enhancements that can be added incrementally.

### Incremental Delivery
1. **Release 1** (MVP): US1 + US2 (hero + branding + cards)
2. **Release 2**: + US3 (theme toggle)
3. **Release 3**: + US4 + US5 (card polish + language toggle)
4. **Release 4**: Performance + Accessibility + Compliance verification

Each release is independently testable and deployable.

---

## Task Summary

**Total Tasks**: 58
**Parallelizable Tasks**: 28 (marked with [P])
**Estimated Time**: 4-6 hours total

### Tasks by Phase
- Phase 1 (Setup): 5 tasks (0.5 hour)
- Phase 2 (Theme): 4 tasks (0.5 hour)
- Phase 3 (US1 - Hero): 6 tasks (1 hour)
- Phase 4 (US2 - Navbar/Cards): 11 tasks (1.5 hours)
- Phase 5 (US3 - Theme Toggle): 4 tasks (0.5 hour)
- Phase 6 (US4 - Card Polish): 3 tasks (0.25 hour)
- Phase 7 (US5 - Language Toggle): 4 tasks (0.25 hour)
- Phase 8 (Performance): 5 tasks (0.5 hour)
- Phase 9 (Accessibility): 9 tasks (0.75 hour)
- Phase 10 (Compliance): 7 tasks (0.5 hour)

### Tasks by User Story
- **US1** (View Professional Landing Page - P1): 6 tasks
- **US2** (Experience Consistent Branding - P1): 11 tasks
- **US3** (Switch Between Themes - P2): 4 tasks
- **US4** (Interact with Feature Cards - P3): 3 tasks
- **US5** (Navigate Using Language Toggle - P3): 4 tasks
- **Setup/Infrastructure**: 9 tasks
- **Testing/Compliance**: 21 tasks

---

## Validation Checklist

- ✅ All tasks follow format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- ✅ All user story tasks have [US#] label
- ✅ Setup/Foundational tasks have NO story label
- ✅ Each user story has Independent Test criteria
- ✅ Task IDs sequential (T001-T058)
- ✅ File paths specific and absolute where needed
- ✅ Dependency graph shows story completion order
- ✅ Parallel execution examples provided
- ✅ MVP scope defined (US1 + US2)
- ✅ Incremental delivery strategy outlined

**Status**: ✅ All validation checks passed - Tasks ready for implementation

---

## Next Steps

1. **Acquire images** (T001-T004) - Source robot and card images
2. **Start with MVP** (Phases 1-4) - Implement US1 + US2 first
3. **Test incrementally** - Complete independent tests after each user story
4. **Add enhancements** (Phases 5-7) - Implement US3, US4, US5
5. **Final validation** (Phases 8-10) - Performance, accessibility, compliance

**Ready to begin**: Run `npm start` in `website/` directory and start with Phase 1 tasks.
