---
id: 002
title: Ocean Sapphire Research - Technology Decisions
stage: research
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-ocean-sapphire-theme
branch: 003-ocean-sapphire-theme
user: developer
command: research
labels: ["research", "ocean-sapphire", "docusaurus", "three.js", "visual-regression", "css-variables", "browser-compatibility"]
links:
  spec: /mnt/e/First_Hackathon/specs/003-ocean-sapphire-theme/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /mnt/e/First_Hackathon/specs/003-ocean-sapphire-theme/research.md
tests:
  - Web research completed for 5 technology questions
---

## Prompt

Research and provide definitive answers for these Ocean Sapphire Design System technology questions. Return findings in a structured format ready for research.md:

1. **Three.js Integration with Docusaurus**: How to embed Three.js scene in custom Docusaurus page without conflicting with SSR (Server-Side Rendering)? What's the recommended pattern for client-only rendering?

2. **Glassmorphism Browser Compatibility**: What's the best fallback strategy for `backdrop-filter: blur(10px)` in browsers that don't support it (Firefox <103, Safari <15.4)? Should we use `@supports` or feature detection?

3. **Visual Regression Testing**: Compare Playwright visual comparisons vs Percy vs Chromatic for Docusaurus projects. Which is best for CI/CD with GitHub Actions considering cost (free tier), ease of setup, and reliability?

4. **CSS Variable Fallback Pattern**: What's the correct CSS syntax for inline fallback values when using CSS variables? Should it be `color: #b8d4ff; color: var(--ocean-text-pale);` or something else?

5. **Docusaurus Theme Swizzling**: Which specific Docusaurus components should be swizzled to customize TOC sidebar active state and code block copy button for Ocean Sapphire theme?

For each question, provide:
- **Decision**: Final recommended approach
- **Rationale**: Why this is the best choice (2-3 sentences)
- **Alternatives Considered**: What else was evaluated and why rejected
- **Implementation Notes**: Specific code patterns or configuration examples

Return this as complete research.md content following the Spec-Kit Plus format.

## Response snapshot

Conducted comprehensive web research on all 5 technology questions and created complete research.md file for Ocean Sapphire Design System. Research covers:

### Decision 1: Three.js Integration with Docusaurus (SSR-Safe Pattern)
- **Decision**: Use Docusaurus `<BrowserOnly>` component to wrap Three.js scenes
- **Rationale**: Prevents SSR conflicts by ensuring Three.js only executes in browser after React hydration
- **Implementation**: Wrapper pattern with fallback content, lazy import using `require()` inside BrowserOnly callback
- **Sources**: Docusaurus official docs, real-world implementations from 2025

### Decision 2: Glassmorphism Browser Compatibility (Fallback Strategy)
- **Decision**: Use `@supports` feature query with solid semi-transparent backgrounds for browsers without backdrop-filter
- **Rationale**: Progressive enhancement, graceful degradation without JavaScript overhead
- **Implementation**: Fallback uses 0.7-0.8 opacity; glassmorphism uses 0.2-0.3 opacity with blur
- **Browser Support Matrix**: Chrome 76+, Firefox 103+, Safari 15.4+ support blur; older browsers get solid fallback
- **Sources**: MDN, Can I Use database, CSS-Tricks

### Decision 3: Visual Regression Testing (Playwright with GitHub Actions)
- **Decision**: Use Playwright's built-in visual comparison with GitHub Actions (free tier)
- **Rationale**: Completely free, open-source, adequate for Ocean Sapphire's focused testing needs (landing page + 3 chapter samples)
- **Alternatives Rejected**: Percy (costs money, unnecessary features), Chromatic (optimized for Storybook/component testing)
- **Implementation**: Playwright CLI auto-generates GitHub Actions workflow, screenshot baselines committed to Git
- **Best Practices**: Use Chromium only, set maxDiffPixels threshold, generate baselines in CI environment
- **Sources**: Playwright docs, GitHub Actions examples, Percy vs Chromatic comparisons

### Decision 4: CSS Variable Fallback Pattern (Inline Dual Declaration)
- **Decision**: Use inline fallback pattern: `color: #b8d4ff; color: var(--ocean-text-pale);`
- **Rationale**: W3C-recommended pattern for graceful degradation in browsers without CSS variable support (IE 11)
- **Why Not `var(--name, fallback)`**: Only works when variable is undefined, not when browser doesn't recognize var() function
- **Implementation**: All Ocean Sapphire CSS variable usage must have hard-coded fallback on line immediately before
- **Variable Naming**: `--ocean-[category]-[variant]` pattern
- **Sources**: MDN, Defensive CSS, W3C specifications

### Decision 5: Docusaurus Component Swizzling (TOC & Code Block Customization)
- **Decision**: Swizzle `TOC` and `CodeBlock/Content` components using "wrap" mode
- **Rationale**: Safe customization without copying entire theme code, easier Docusaurus upgrades
- **Components**: TOC for active state styling (cyan glow, left border), CodeBlock/Content for copy button Ocean Sapphire effects
- **Implementation**: Minimal wrapper components, push styling to CSS files
- **Best Practices**: Prefer wrap over eject, document Docusaurus version, test after upgrades
- **Sources**: Docusaurus swizzling docs, GitHub discussions

### Additional Content
- Complete file structure for Ocean Sapphire implementation
- CSS architecture (4 layers: variables, base styles, components, animations)
- Dependencies to add (three, @types/three, @playwright/test)
- Complete CSS variables reference (15 variables)
- Testing checklist (10 items)
- Implementation notes with code examples for all patterns

### Sources Referenced
- 25+ authoritative sources from MDN, Docusaurus docs, Playwright docs, CSS-Tricks, Can I Use, GitHub discussions
- All sources cited with markdown hyperlinks in research.md
- Research reflects 2025 best practices and current browser support landscape

## Outcome

- ✅ Impact: Complete research.md created at `/mnt/e/First_Hackathon/specs/003-ocean-sapphire-theme/research.md` with 5 major technology decisions, implementation patterns, code examples, and 25+ cited sources
- 🧪 Tests: No tests run (research phase)
- 📁 Files: Created research.md (750+ lines)
- 🔁 Next prompts: Create plan.md (architecture decisions) and tasks.md (implementation breakdown)
- 🧠 Reflection: Research successfully translates Ocean Sapphire spec requirements into concrete, actionable technology decisions with proven patterns from 2025 web development ecosystem

## Evaluation notes (flywheel)

- Failure modes observed: None - web search successfully found authoritative sources for all 5 questions
- Graders run and results (PASS/FAIL): PASS - Research addresses all user questions with decision/rationale/alternatives/implementation structure
- Prompt variant (if applicable): Standard research request format
- Next experiment (smallest change to try): None - research complete and ready for architecture planning phase
