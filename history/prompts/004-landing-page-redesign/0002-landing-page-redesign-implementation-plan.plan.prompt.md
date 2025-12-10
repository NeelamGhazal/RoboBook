---
id: 0002
title: Landing page redesign implementation plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-landing-page-redesign
branch: 004-landing-page-redesign
user: user
command: /sp.plan
labels: ["architecture", "adr", "react", "theme-toggle", "glassmorphism", "responsive-design", "performance"]
links:
  spec: /mnt/e/First_Hackathon/specs/004-landing-page-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-landing-page-redesign/plan.md
  - specs/004-landing-page-redesign/research.md
  - specs/004-landing-page-redesign/data-model.md
  - specs/004-landing-page-redesign/quickstart.md
  - specs/004-landing-page-redesign/contracts/README.md
  - CLAUDE.md (agent context updated)
tests: []
---

## Prompt

/sp.plan

Feature: Redesign RoboBook Landing Page according to Constitution v1.4.0

## Architecture Decisions (ADR Suggestions)

| Decision | Options Considered | Recommendation | Rationale |
|----------|-----------------|----------------|-----------|
| Font Usage | Orbitron everywhere vs. only landing page | Use Orbitron only on navbar + landing page | Matches Constitution §VI. Preserves readability for docs/blog |
| Hero Layout | CSS Grid vs. Flex | Flex with responsive breakpoints | Flex easier for exact 50/50 (desktop) & 60/40 (tablet) layout |
| Robot Image | Animated Three.js vs. static 3D WebP | Static 3D WebP | Constitution §XIII mandates replacement of animation with professional image |
| Theme Toggle Implementation | CSS variables vs. JS state | React state + CSS variables | Allows 200ms transition, WCAG AA compliance, layout unchanged (Constitution §XX) |
| Feature Cards Layout | Flex row vs. Grid | Flex row with image left / text right | Maintains 40/60 split; easier to animate shimmer effect |
| Shimmer Animation | Keyframes vs. JS animation | CSS keyframes | GPU-accelerated, smooth 3s infinite loop; no JS overhead (Constitution §XXI) |

## Component Architecture & Responsibilities

### Navbar (`src/components/Navbar.tsx`)
- Fixed height: 72px
- Left: Logo (40×40px) + RoboBook text (Orbitron 22px)
- Right: Navigation links (`Textbook | Blog`) 17px, gap 32px, Language toggle EN/UR, Theme toggle (28px, 24px spacing), GitHub icon
- Styling: Ocean Sapphire colors + backdrop blur
- Hover effects: Logo & links cyan glow

### Hero Section (`src/components/Hero/*`)
- Left (Text):
  - Heading: Orbitron, 4.5rem, line-height 1.2, max-width 580px
  - Paragraph: Georgia serif, 1.3rem, max-width 520px
  - Button: 200×60px, Orbitron 1.2rem, cyan border + glow on hover
  - Padding: top 100px, side min 24px
- Right (Robot Image):
  - Static 3D half-body robot, width 520–580px, height auto
  - Responsive: desktop 50%, tablet 40%, mobile full width
  - Lazy loading, WebP, <200KB

### Features Section (`src/components/FeatureCards/*`)
- Section:
  - Max-width: 1200px, centered
  - Heading: Georgia serif, 2.8rem, mb-6
  - Description: 1.1rem, max-width 900px, mb-16
- Cards (3):
  - Flex row (desktop), stacked mobile
  - Left: Image 40% width, min 180px, 3:2 aspect ratio, WebP <50KB
  - Right: Text 60%, title Orbitron 1.5rem, text Georgia 1rem
  - Glassmorphism: rgba(0,50,100,0.3) + blur 10px
  - Border: 1px solid rgba(0,150,255,0.3), border-radius 12px
  - Hover: shimmer 3s infinite
  - Gap: 24px desktop, 16px mobile

## Development Steps

1. Setup
   - Import Orbitron font for landing page + navbar only
   - Import WebP robot + feature images, lazy-load enabled
   - Define CSS variables for Ocean Sapphire Dark + Light themes

2. Navbar
   - Implement fixed navbar (72px) + layout
   - Add language toggle (EN/UR visual only)
   - Add theme toggle React state (200ms transition)
   - Test hover glow & spacing compliance

3. Hero Section
   - Left panel text + button styling
   - Right panel robot image: responsive, lazy-loaded
   - Implement 50/50 desktop, 60/40 tablet, stacked mobile layout
   - Apply precise paddings & gaps

4. Features Section
   - Heading + description
   - Feature cards: flex row, 40/60 image-text, glassmorphism
   - Hover shimmer effect (CSS keyframes)
   - Responsive stack on mobile
   - Lazy-load images

5. Theme Toggle
   - Define Dark + Light palettes as CSS variables
   - Implement React state toggle
   - Animate transition 200ms, verify WCAG AA contrast

6. Performance & Optimization
   - Test page load <3s
   - Verify CLS = 0 (no layout shift)
   - Ensure shimmer & hover effects GPU-accelerated
   - Images compressed: robot <200KB, cards <50KB

7. Testing
   - Breakpoints: 1024px (desktop), 768px (tablet), 640px (mobile)
   - Accessibility: keyboard nav, color contrast
   - Console errors: none
   - Verify Orbitron applied only to landing page + navbar

## Deliverables
- Landing page `/index.tsx` fully implemented
- Hero section components
- Features section components
- Custom CSS (`custom.css`) with theme + glassmorphism + typography
- React theme toggle with 200ms transition
- All images optimized + lazy-loaded
- Responsive layout verified
- WCAG AA compliance confirmed
- No console errors

## Dependencies & Risks
| Risk | Mitigation |
|------|-----------|
| Images too large → slow load | Use WebP + compression, lazy-load |
| Theme toggle CSS issues | Use CSS variables + React state |
| Shimmer animation heavy | Use GPU-accelerated keyframes, test on mid-range device |
| Responsive breakpoints mismatch | Tailwind breakpoints: 640px, 768px, 1024px; test manually |
| Font conflict | Orbitron imported only for landing page & navbar |

## Response snapshot

Successfully completed `/sp.plan` with comprehensive implementation plan for RoboBook landing page redesign.

**Phase 0: Research (Complete)**:
Resolved 7 research areas:
1. React theme toggle implementation (useState + CSS variables with data-theme attribute)
2. Glassmorphism browser compatibility (backdrop-filter with -webkit- prefix, @supports fallback)
3. Lazy loading strategy (native loading="lazy" with explicit width/height)
4. Responsive breakpoints (custom media queries 768px, 1024px overriding Docusaurus defaults)
5. Shimmer animation performance (CSS keyframes with ::before pseudo-element, GPU-accelerated)
6. Orbitron font scoping (load via Google Fonts in landing components only, inline styles)
7. Image asset specifications (robot 520-580px/<200KB WebP, cards 3:2/<50KB WebP)

**Phase 1: Design & Contracts (Complete)**:
- **Data Model**: 5 entities defined (Theme State, Hero Content, Feature Card, Feature Cards Collection, Navbar Config) with TypeScript interfaces and validation rules
- **Contracts**: N/A (frontend-only, no APIs per Constitution §XXII)
- **Quickstart Guide**: 8-phase implementation guide (4-6 hours estimated) with detailed testing checklist

**Architecture Decisions**:
Documented 6 ADRs capturing technical choices:
- ADR-001: Font Usage Strategy (Orbitron navbar+landing only)
- ADR-002: Hero Layout Implementation (Flexbox vs Grid)
- ADR-003: Robot Image Strategy (Static WebP vs Animation)
- ADR-004: Theme Toggle Implementation (React state + CSS variables)
- ADR-005: Feature Cards Layout (Flexbox 40/60 split)
- ADR-006: Shimmer Animation Implementation (CSS keyframes)

**Constitution Check**: ✅ All 9 gates passed (File Restrictions, Brand Consistency, Typography Restrictions, Navbar Specs, Hero Specs, Feature Cards Specs, Theme Toggle, Performance Standards, Non-Negotiable Restrictions). Post-design re-evaluation confirmed no violations, high confidence in constitution compliance.

**Agent Context Updated**: Added TypeScript 5.x, React 18+, Node.js 18+, Docusaurus 3.x to CLAUDE.md via update-agent-context.sh script.

**Deliverables Created**:
- plan.md (470 lines): Technical context, constitution checks, ADRs, phases, risks, metrics
- research.md (7 research areas with findings, decisions, alternatives)
- data-model.md (5 entities, TypeScript interfaces, CSS variables, data flow diagram)
- quickstart.md (8 phases, 45 checkboxes, troubleshooting, quick reference)
- contracts/README.md (explains N/A status for frontend-only project)

## Outcome

- ✅ Impact: Completed comprehensive implementation plan with 6 documented ADRs, 7 resolved research areas, 5 data entities, detailed quickstart guide, constitution compliance verification; ready for `/sp.tasks` command to generate 20-30 granular implementation tasks
- 🧪 Tests: No tests (planning phase, quickstart includes testing checklist for implementation phase)
- 📁 Files: specs/004-landing-page-redesign/plan.md (created), research.md (created), data-model.md (created), quickstart.md (created), contracts/README.md (created), CLAUDE.md (updated)
- 🔁 Next prompts: Acquire image assets (robot 520-580px/<200KB, cards 3×3:2/<50KB), run /sp.tasks to generate tasks.md, begin implementation following quickstart 8-phase guide, complete testing checklist (Lighthouse, WCAG, responsive), create pull request with screenshots
- 🧠 Reflection: User-provided architecture decisions accelerated planning phase significantly. All 6 ADRs directly sourced from user input with rationale expanded. Constitution v1.4.0 provides complete specifications (navbar dimensions, typography sizes, theme palettes, performance targets) eliminating clarification needs. Frontend-only scope simplifies architecture (no API contracts, no persistent storage, React state only). Quickstart guide bridges plan-to-implementation gap with actionable 45-item checklist.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution Check (9/9 gates PASS - initial and post-design)
- Prompt variant (if applicable): User provided pre-filled architecture decisions table, accelerating ADR documentation
- Next experiment (smallest change to try): Generate tasks.md via /sp.tasks, verify task breakdown aligns with quickstart phases (expect 20-30 tasks across setup → theme → hero → cards → navbar → integration → testing sequence)
