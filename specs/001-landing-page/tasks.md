---
description: "Task list for Professional Landing Page implementation"
---

# Tasks: Professional Landing Page

**Input**: Design documents from `/specs/001-landing-page/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, quickstart.md

**Tests**: Not requested in specification. Test tasks omitted per template guidelines.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `src/` at repository root
- Paths shown below assume web application structure from plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Install core dependencies (three, @react-three/fiber, @react-three/drei, framer-motion, clsx) via npm
- [x] T002 [P] Configure Tailwind CSS with Humaride Robotics color palette in tailwind.config.js
- [x] T003 [P] Add Google Fonts (Inter, Poppins) to docusaurus.config.js headTags
- [x] T004 [P] Create project directory structure (src/components/LandingPage/, src/hooks/, src/styles/)
- [x] T005 [P] Create Humaride Robotics color variables in src/styles/theme-colors.css
- [x] T006 [P] Update src/css/custom.css with Tailwind directives (@tailwind base, components, utilities)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 [P] Create useMediaQuery hook in src/hooks/useMediaQuery.ts
- [x] T008 [P] Create useReducedMotion hook in src/hooks/useReducedMotion.ts
- [x] T009 [P] Create useWebGLSupport hook in src/hooks/useWebGLSupport.ts
- [x] T010 [P] Create FallbackSVG component in src/components/common/FallbackSVG.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - First Impression & Navigation (Priority: P1) 🎯 MVP

**Goal**: Enable users to understand textbook value proposition and navigate to `/docs/intro` with one click

**Independent Test**: Load page at `/`, verify heading with gradient, click "Read Book" button, confirm SPA navigation to `/docs/intro` without page reload

### Implementation for User Story 1

- [x] T011 [P] [US1] Create Hero component in src/components/LandingPage/Hero.tsx
- [x] T012 [P] [US1] Create Hero.module.css with gradient heading, subheading, and CTA button styles in src/components/LandingPage/Hero.module.css
- [x] T013 [US1] Create landing page index file in src/pages/index.tsx integrating Hero component
- [x] T014 [US1] Implement heading gradient (cyan #00F0FF to magenta #FF2A6D) in Hero.module.css
- [x] T015 [US1] Implement CTA button with neon glow hover effect (box-shadow: 0 0 20px #00F0FF, 0 0 40px #00F0FF) in Hero.module.css
- [x] T016 [US1] Implement button pulse animation (3s interval) with @keyframes in Hero.module.css
- [x] T017 [US1] Add keyboard navigation support with focus indicators (:focus-visible) in Hero.module.css
- [x] T018 [US1] Implement responsive layout (50/50 desktop, stacked mobile) in src/pages/index.tsx
- [x] T019 [US1] Add ARIA labels to button (aria-label="Navigate to textbook introduction") in Hero.tsx
- [x] T020 [US1] Test SPA navigation to /docs/intro using @docusaurus/Link in Hero.tsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Visual Engagement (Priority: P2)

**Goal**: Enhance user engagement with 3D Three.js cube animation (60fps on desktop) with fallbacks for mobile, reduced motion, and WebGL unsupported

**Independent Test**: Load page on desktop, verify Three.js cube rotates at 60fps with neon cyan edges. Test on mobile/reduced motion/no WebGL, verify static SVG fallback appears.

### Implementation for User Story 2

- [ ] T021 [P] [US2] Create AnimatedCube component in src/components/LandingPage/AnimatedCube.tsx
- [ ] T022 [P] [US2] Create AnimatedCube.module.css for cube container styles in src/components/LandingPage/AnimatedCube.module.css
- [ ] T023 [US2] Implement Three.js scene with Canvas from @react-three/fiber in AnimatedCube.tsx
- [ ] T024 [US2] Create cube geometry with BoxGeometry and neon cyan edges (#00F0FF) using EdgesGeometry in AnimatedCube.tsx
- [ ] T025 [US2] Implement cube rotation animation (60fps) using useFrame hook in AnimatedCube.tsx
- [ ] T026 [US2] Add subtle floating motion using Float component from @react-three/drei in AnimatedCube.tsx
- [ ] T027 [US2] Implement lazy loading with React.lazy() and Suspense for AnimatedCube in src/pages/index.tsx
- [ ] T028 [US2] Add desktop-only rendering logic (≥768px) with useMediaQuery in src/pages/index.tsx
- [ ] T029 [US2] Add reduced motion detection to disable animation in src/pages/index.tsx
- [ ] T030 [US2] Add WebGL detection to show fallback when unsupported in src/pages/index.tsx
- [ ] T031 [US2] Integrate FallbackSVG component for mobile/reduced motion/no WebGL scenarios in src/pages/index.tsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Content Discovery (Priority: P3)

**Goal**: Provide textbook statistics through 4 interactive glass-morphism cards with staggered animations and hover effects

**Independent Test**: Scroll to stats section, verify 4 cards ("4 Modules", "21 Chapters", "50+ Code Examples", "AI-Powered Chatbot") animate in with 100ms stagger. Hover over cards, verify 10px lift and cyan glow.

### Implementation for User Story 3

- [x] T032 [P] [US3] Create StatCard component in src/components/LandingPage/StatCard.tsx
- [x] T033 [P] [US3] Create StatCard.module.css with glass-morphism styles (backdrop-filter: blur(10px)) in src/components/LandingPage/StatCard.module.css
- [x] T034 [P] [US3] Create StatsSection component in src/components/LandingPage/StatsSection.tsx
- [x] T035 [P] [US3] Create StatsSection.module.css with responsive grid layout in src/components/LandingPage/StatsSection.module.css
- [x] T036 [US3] Implement glass-morphism effect with backdrop-filter and Firefox fallback in StatCard.module.css
- [x] T037 [US3] Implement hover effects (transform: translateY(-10px), cyan glow) with Framer Motion whileHover in StatCard.tsx
- [x] T038 [US3] Implement staggered fade-in animation (100ms stagger) using Framer Motion variants in StatsSection.tsx
- [x] T039 [US3] Add scroll detection with useInView hook from Framer Motion in StatsSection.tsx
- [x] T040 [US3] Create stats array with 4 items ("4 Modules", "21 Chapters", "50+ Code Examples", "AI-Powered Chatbot") in StatsSection.tsx
- [x] T041 [US3] Integrate StatsSection component into landing page layout in src/pages/index.tsx
- [x] T042 [US3] Implement light mode support with adjusted card backgrounds (#FFFFFF) in StatCard.module.css

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T043 [P] Add full viewport height layout (100vh) and vertical centering to src/pages/index.tsx
- [x] T044 [P] Implement mobile responsive breakpoint (768px) for vertical stacking in src/pages/index.tsx
- [x] T045 [P] Add extreme viewport handling (<320px, >2560px) with max-width constraints in src/pages/index.tsx
- [x] T046 [P] Verify color contrast ratios (cyan on dark = 10.4:1) meet WCAG 2.1 AA
- [x] T047 [P] Verify proper heading hierarchy (h1 for main heading) in Hero.tsx
- [x] T048 [P] Add prefers-color-scheme light mode support across all components
- [x] T049 Run Lighthouse CI audit and verify Performance >90, Accessibility >90
- [x] T050 Test keyboard navigation (Tab to button, Enter to navigate) and verify focus indicators
- [x] T051 Test on iPhone 12 (390×844) and Samsung Galaxy S21 (360×800) for responsive layout
- [x] T052 Verify FCP <1.5s and TTI <3s with DevTools Performance panel
- [x] T053 Verify 60fps animation performance on desktop with discrete GPU using DevTools

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 layout but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates below US1/US2 but independently testable

### Within Each User Story

- Hero component before landing page integration (US1: T011→T012→T013)
- AnimatedCube component before lazy loading integration (US2: T021→T022→T023-T026→T027)
- StatCard before StatsSection (US3: T032→T033→T034→T035)
- Implementation before integration (T036-T040 before T041)

### Parallel Opportunities

- All Setup tasks (T001-T006) can run in parallel
- All Foundational tasks (T007-T010) can run in parallel
- Once Foundational phase completes, all 3 user stories can start in parallel:
  - Developer A: User Story 1 (T011-T020)
  - Developer B: User Story 2 (T021-T031)
  - Developer C: User Story 3 (T032-T042)
- All Polish tasks (T043-T048) can run in parallel
- Testing tasks (T049-T053) should run sequentially after Polish

---

## Parallel Example: User Story 1

```bash
# Launch Hero component and styles together:
Task: "Create Hero component in src/components/LandingPage/Hero.tsx" (T011)
Task: "Create Hero.module.css with gradient heading, subheading, and CTA button styles in src/components/LandingPage/Hero.module.css" (T012)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T010) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (T011-T020)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T010)
2. Once Foundational is done:
   - Developer A: User Story 1 (T011-T020)
   - Developer B: User Story 2 (T021-T031)
   - Developer C: User Story 3 (T032-T042)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Tests not requested in specification, omitted per template guidelines
