---
id: 0002
title: Create landing page implementation plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-landing-page
branch: 001-landing-page
user: developer
command: /sp.plan
labels: ["landing-page", "architecture", "react-three-fiber", "framer-motion", "performance"]
links:
  spec: specs/001-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-landing-page/plan.md
  - specs/001-landing-page/research.md
  - specs/001-landing-page/quickstart.md
  - CLAUDE.md
tests:
  - none
---

## Prompt

```text
/sp.plan
```

## Response snapshot

Successfully created implementation plan with complete technical architecture:

**Summary**: Landing page with 50/50 split layout (desktop), Three.js animated cube (left), hero content (right), 4 glass-morphism stat cards (below). Performance targets: <3s load, FCP <1.5s, TTI <3s, 60fps animations, Lighthouse >90.

**Technical Stack**:
- TypeScript 5.x + React 18+ + Node.js 18+
- Docusaurus 3.x (core framework)
- Three.js 0.160+ with @react-three/fiber + @react-three/drei
- Framer Motion 10+ (UI animations)
- Tailwind CSS 3.x + CSS Modules (hybrid styling)
- Jest + React Testing Library + Playwright + Lighthouse CI

**Constitution Check**: ✅ PASSED all applicable principles
- Principle V (Performance & Accessibility): FCP <1.5s, TTI <3s, keyboard navigation, ARIA labels, color contrast 10.4:1, reduced motion support
- Principle VI (Brand Consistency): Humaride Robotics color palette (#00F0FF cyan, #FF2A6D magenta), Inter/Poppins fonts, glass-morphism styling

**Project Structure**: Web application with Docusaurus conventions
- `src/pages/index.tsx` (landing page root)
- `src/components/LandingPage/` (Hero, AnimatedCube, StatCard, StatsSection)
- `src/hooks/` (useMediaQuery, useReducedMotion, useWebGLSupport)
- Co-located CSS Modules for component isolation
- Tests mirror source structure

**Phase 0 Research (research.md)**: Resolved 7 technical decisions
1. Docusaurus custom pages: Use `src/pages/index.tsx` with `@docusaurus/Link` for SPA navigation
2. Three.js + React: @react-three/fiber (declarative) + @react-three/drei (helpers) with lazy loading
3. Framer Motion: `useInView` + staggerChildren (100ms) + `whileHover` for card animations
4. Glass-morphism: `backdrop-filter: blur(10px)` with Firefox <103 fallback
5. Tailwind + CSS Modules: Hybrid approach (Tailwind for layout, CSS Modules for complex styles)
6. Accessibility: Focus indicators, ARIA labels, keyboard nav, reduced motion detection
7. Performance: Code splitting, lazy loading, critical CSS inlining, Lighthouse CI

**Phase 1 Quickstart (quickstart.md)**: Developer setup guide with code samples
- Dependencies: `npm install three @react-three/fiber @react-three/drei framer-motion clsx`
- Tailwind config with Humaride Robotics color palette
- Custom hooks implementation (useMediaQuery, useReducedMotion, useWebGLSupport)
- Hero, StatCard, StatsSection component boilerplate with CSS Modules
- Testing setup (Jest, Playwright, Lighthouse CI)
- Validation checklist and troubleshooting

**ADR Candidates**: 2 architectural decisions flagged for documentation
1. Three.js + React-Three-Fiber vs Vanilla Three.js (significant: long-term maintainability)
2. Desktop-Only 3D Animation (significant: performance/bundle size tradeoff)

## Outcome

- ✅ Impact: Complete implementation plan created with technical context, constitution compliance, project structure, Phase 0 research (7 decisions), Phase 1 quickstart guide. No violations. Ready for `/sp.tasks`.
- 🧪 Tests: No automated tests (planning document). Manual validation: constitution check passed all gates, research resolved all NEEDS CLARIFICATION, quickstart provides runnable code samples.
- 📁 Files: Created plan.md (144 lines), research.md (350+ lines with 7 research topics), quickstart.md (400+ lines with setup/dev/test guides). Updated CLAUDE.md with TypeScript/React/Node.js context.
- 🔁 Next prompts: Ready for `/sp.tasks` to generate actionable task list. Consider `/sp.adr` for React-Three-Fiber and Desktop-Only Animation decisions after user review.
- 🧠 Reflection: Plan balances performance (lazy loading, code splitting) with wow factor (3D cube, glass-morphism). Constitution compliance clean (no violations). Research phase comprehensive (covered 7 technical unknowns with alternatives/rationale). Quickstart enables fast developer onboarding.

## Evaluation notes (flywheel)

- Failure modes observed: None. All constitution gates passed. No NEEDS CLARIFICATION remained unresolved. Quickstart provides complete setup path.
- Graders run and results (PASS/FAIL): Constitution Check PASS - Principle V (Performance & Accessibility) and VI (Brand Consistency) fully compliant. Research PASS - all 7 topics resolved with decision/rationale/alternatives. Quickstart PASS - runnable code samples, clear validation checklist.
- Prompt variant (if applicable): Standard planning workflow with Phase 0 (research) and Phase 1 (quickstart) execution.
- Next experiment (smallest change to try): Proceed with `/sp.tasks` to validate plan completeness during task breakdown. Monitor for missing technical details or implementation ambiguities.
