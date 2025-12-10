# Feature Specification: Ocean Sapphire Design System

**Feature Branch**: `003-ocean-sapphire-theme`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Apply Ocean Sapphire Design System to Landing Page and Book Reader"

## Clarifications

### Session 2025-12-10

- Q: How should developers verify Ocean Sapphire is rendering correctly during development and in CI/CD pipelines? → A: Visual regression testing with screenshot comparison + automated CSS validation for variables and contrast ratios
- Q: Should Ocean Sapphire be feature-flagged for gradual user rollout or deployed immediately as the default theme? → A: Immediate full replacement - Ocean Sapphire becomes default theme for all users in single deployment (no feature flags)
- Q: What should happen if Fira Code fails to load from CDN or takes too long? → A: CSS font-family stack fallback: 'Fira Code', 'Courier New', Consolas, monospace (automatic, no detection)
- Q: Does a custom landing page already exist, or do we need to create one from scratch? → A: Create new custom landing page from scratch at website/src/pages/index.tsx (larger scope, includes markup + styling)
- Q: Should older browsers without CSS variable support receive fallback styling or be completely unsupported? → A: Graceful degradation with inline fallback values using `color: #b8d4ff; color: var(--ocean-text-pale);` pattern

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Landing Page Ocean Sapphire Theme (Priority: P1)

A visitor arrives at the textbook landing page and experiences a premium, professional design with deep ocean blue aesthetics, smooth animations, and clear branding that establishes credibility for the educational content.

**Why this priority**: The landing page is the first impression and primary entry point. A professional, accessible design directly impacts user trust and engagement. This story delivers immediate visual value independent of content pages.

**Scope Note**: This story includes creating the custom landing page component from scratch at `website/src/pages/index.tsx` with Ocean Sapphire markup structure, Three.js scene integration, and complete Ocean Sapphire styling.

**Independent Test**: Can be fully tested by visiting the root URL `/`, verifying the Ocean Sapphire color palette is applied, glassmorphism cards render correctly, shimmer animations trigger on hover, and the "Read Book" button has cyan glow effects. Delivers value by providing a polished, trustworthy first impression.

**Acceptance Scenarios**:

1. **Given** a visitor navigates to the landing page, **When** the page loads, **Then** the background displays the Ocean Sapphire gradient (`#001529` → `#002140` → `#003a6d`) and all text uses Georgia serif typography
2. **Given** the landing page is displayed, **When** a visitor hovers over a stats card, **Then** the card displays a shimmer animation (3-second gradient sweep) with glassmorphism effects (`backdrop-filter: blur(10px)`)
3. **Given** the landing page is loaded, **When** a visitor hovers over the "Read Book" button, **Then** the button shows a cyan glow effect (`0 0 20px rgba(0, 150, 255, 0.5)`)
4. **Given** the Three.js scene is rendered, **When** the animation runs, **Then** the geometric shape displays blue gradient edges transitioning from `#0096ff` to `#00d4ff` at 60fps
5. **Given** the landing page is viewed on mobile (iPhone 12 or Galaxy S21), **When** the viewport is resized, **Then** all Ocean Sapphire elements remain visually correct and interactive

---

### User Story 2 - Book Reader Pages Ocean Sapphire Theme (Priority: P2)

A learner navigates through textbook chapters and experiences consistent Ocean Sapphire styling with comfortable reading typography, smooth page transitions, and clear visual hierarchy that enhances comprehension during long reading sessions.

**Why this priority**: After the landing page attracts users, the book reader experience determines engagement and learning effectiveness. Ocean Sapphire's high-contrast pale blue text on dark backgrounds reduces eye strain for extended reading. This story is independently valuable for existing users.

**Independent Test**: Can be fully tested by navigating to any `/docs/*` page, verifying Georgia serif typography for body text, TOC sidebar active state highlighting with cyan glow, smooth fade-in animations, and WCAG AAA contrast ratios. Delivers value by providing comfortable long-form reading experience.

**Acceptance Scenarios**:

1. **Given** a learner navigates to any chapter page, **When** the page loads, **Then** the background is `#0d1117` with subtle blue ambient glow, headings use Georgia serif in white, and body text is Georgia 16-18px in `#b8d4ff` with line-height 2.0
2. **Given** a chapter page is displayed, **When** a learner clicks a TOC sidebar link, **Then** the active chapter shows a left border (`3px solid #0096ff`) with cyan glow and the page transition includes a fade-in animation (0.6s ease)
3. **Given** a chapter contains code blocks, **When** the learner views the code, **Then** the code uses Fira Code monospace font, preserves syntax highlighting, and the copy button has a cyan hover effect
4. **Given** a chapter has highlight boxes (key insights), **When** displayed, **Then** the box has background `rgba(0, 100, 200, 0.15)`, left border `4px solid #0096ff`, and includes a 💎 icon in cyan color
5. **Given** a learner scrolls through a chapter, **When** content blocks enter the viewport, **Then** they reveal with opacity and transform animations (scroll-triggered)

---

### User Story 3 - Chapter Card Glassmorphism Effects (Priority: P3)

A learner browses chapter listings or module overviews and interacts with chapter cards that feature modern glassmorphism effects and shimmer animations, creating a premium feel that reinforces the quality of the educational content.

**Why this priority**: Chapter cards provide visual organization and navigation cues. Glassmorphism and shimmer effects differentiate the textbook from standard documentation sites, supporting the "AI-native premium educational resource" positioning. This can be tested independently on any page with chapter cards.

**Independent Test**: Can be fully tested by navigating to pages with chapter cards, hovering over them to trigger shimmer animations, and verifying glassmorphism rendering (`rgba(0, 50, 100, 0.3)` background with `blur(10px)`). Delivers value by enhancing perceived quality and interactivity.

**Acceptance Scenarios**:

1. **Given** a page displays chapter cards, **When** a learner hovers over a card, **Then** the card displays a shimmer animation (3-second infinite gradient sweep from left to right)
2. **Given** chapter cards are rendered, **When** viewed, **Then** each card has glassmorphism styling with background `rgba(0, 50, 100, 0.3)`, `backdrop-filter: blur(10px)`, border `1px solid rgba(0, 150, 255, 0.3)`, and border-radius `12px`
3. **Given** chapter badges are displayed, **When** viewed, **Then** each badge has background `rgba(0, 150, 255, 0.2)`, border `1px solid #0096ff`, text color `#00d4ff`, and cyan glow effect `0 0 15px rgba(0, 150, 255, 0.3)`
4. **Given** the learner is on a mobile device, **When** viewing chapter cards, **Then** touch interactions work smoothly and glassmorphism effects render correctly

---

### User Story 4 - CSS Variable System and Theme Consistency (Priority: P4)

Developers and designers maintain the Ocean Sapphire theme across all pages using a centralized CSS variable system in `:root`, enabling consistent styling and easy future theme adjustments without modifying component-level styles.

**Why this priority**: A CSS variable system prevents style drift and makes the theme maintainable. While less visible to end users, it's critical for long-term quality. This story ensures technical foundation but is lower priority than user-facing visual changes.

**Independent Test**: Can be fully tested by inspecting the compiled CSS, verifying all Ocean Sapphire colors and design tokens are defined as CSS variables in `:root`, and confirming all components reference these variables. Delivers value by reducing technical debt and enabling rapid theme iterations.

**Acceptance Scenarios**:

1. **Given** the Docusaurus site is built, **When** inspecting the compiled `custom.css`, **Then** all Ocean Sapphire colors are defined as CSS variables in `:root` (e.g., `--ocean-primary-deep: #001529`, `--ocean-accent-cyan: #0096ff`)
2. **Given** CSS variables are defined, **When** any component needs Ocean Sapphire colors, **Then** it references the CSS variable (e.g., `color: var(--ocean-text-pale)`) rather than hard-coded hex values
3. **Given** dark and light mode support is required, **When** the theme toggles, **Then** CSS variables update via data attributes or media queries maintaining WCAG AAA contrast in both modes
4. **Given** a developer needs to adjust a color, **When** they update a single CSS variable in `:root`, **Then** the change propagates across all components using that variable

---

### Edge Cases

- What happens when a browser doesn't support `backdrop-filter` (glassmorphism)? System should provide fallback styling with solid semi-transparent backgrounds maintaining readability.
- How does the system handle users with `prefers-reduced-motion` enabled? Shimmer animations and page transitions should be disabled or simplified to respect accessibility preferences.
- What happens when Three.js fails to load or GPU acceleration is unavailable? Landing page should display a static fallback image or CSS gradient background without breaking layout.
- How does the TOC sidebar handle very long chapter titles with Ocean Sapphire styling? Text should truncate with ellipsis while maintaining hover tooltip for full title.
- What happens on extremely small screens (<360px wide) with glassmorphism effects? Layout should remain functional with appropriately scaled cards and readable text.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST create a new custom landing page React component at `website/src/pages/index.tsx` with TypeScript, including markup structure for Ocean Sapphire hero section, stats cards, Three.js scene container, and call-to-action button
- **FR-002**: Landing page MUST display Ocean Sapphire gradient background (`linear-gradient(180deg, #001529 0%, #002140 50%, #003a6d 100%)`) covering the full viewport
- **FR-003**: Landing page heading "HUMARIDE" MUST use Georgia font family, 4.5em size, white color with cyan glow (`text-shadow: 0 0 30px rgba(0, 150, 255, 0.6)`)
- **FR-004**: Stats cards MUST implement glassmorphism with `background: rgba(0, 50, 100, 0.3)`, `backdrop-filter: blur(10px)`, and `border: 1px solid rgba(0, 150, 255, 0.3)`
- **FR-005**: Stats cards MUST display shimmer animation on hover (3-second infinite gradient sweep from left to right)
- **FR-006**: "Read Book" button MUST have `background: rgba(0, 150, 255, 0.2)`, `border: 1px solid #0096ff`, and on hover show `box-shadow: 0 0 20px rgba(0, 150, 255, 0.5)`
- **FR-007**: Three.js animated scene MUST render with blue gradient edges transitioning from `#0096ff` to `#00d4ff` at minimum 60fps
- **FR-008**: Book reader pages (`/docs/*`) MUST have background `#0d1117` with subtle radial gradient ambient glow using `rgba(0, 150, 255, 0.1)`
- **FR-009**: TOC sidebar MUST highlight active chapter with left border `3px solid #0096ff` and glow effect
- **FR-010**: Book reader body text MUST use Georgia font family, 16-18px size, color `#b8d4ff`, and line-height 2.0
- **FR-011**: Chapter page transitions MUST include fade-in animation with slight upward slide (0.6s ease)
- **FR-012**: Code blocks MUST use font-family stack `'Fira Code', 'Courier New', Consolas, monospace` for automatic fallback and add cyan hover effect to copy button
- **FR-013**: Highlight boxes MUST have background `rgba(0, 100, 200, 0.15)`, left border `4px solid #0096ff`, and 💎 icon
- **FR-014**: Chapter badges MUST have background `rgba(0, 150, 255, 0.2)`, border `1px solid #0096ff`, text `#00d4ff`, and glow `0 0 15px rgba(0, 150, 255, 0.3)`
- **FR-015**: System MUST define all Ocean Sapphire colors as CSS variables in `:root` and use inline fallback pattern (e.g., `color: #b8d4ff; color: var(--ocean-text-pale);`) for graceful degradation in browsers without CSS variable support
- **FR-016**: All animations MUST respect `prefers-reduced-motion` media query for accessibility
- **FR-017**: System MUST provide fallback styling for browsers that don't support `backdrop-filter`
- **FR-018**: All interactive elements MUST maintain WCAG AAA contrast ratios (minimum 7:1 for normal text, 4.5:1 for large text)
- **FR-019**: Ocean Sapphire theme MUST be responsive on mobile devices (iPhone 12: 390×844, Galaxy S21: 360×800)

### Key Entities

- **Color Palette**: Ocean Sapphire design system with 10 defined colors (Primary Deep Blue, Mid Ocean Blue, Light Ocean Blue, Accent Cyan, Soft Cyan, Light Cyan Text, Pale Blue Text, Card Background, Border Glow, Shimmer Effect) used consistently across all UI elements
- **Typography System**: Georgia serif for headings and body text with defined hierarchy (H1: 4.5em/300 weight, H2: 2.8em/300 weight, H3: 1.8em/400 weight, Body: 1.1em, Captions: 0.9em italic), font stack `'Fira Code', 'Courier New', Consolas, monospace` for code blocks with automatic fallback
- **Animation Set**: Collection of GPU-accelerated animations (shimmer: 3s infinite gradient sweep, fade-in: 0.6s ease, glow expansion: 0.3s ease, active chapter pulse: 2s infinite) with `prefers-reduced-motion` support
- **Glassmorphism Components**: Card backgrounds using `backdrop-filter: blur(10px)` with semi-transparent backgrounds, 1px borders with cyan glow, and 12px border-radius

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Landing page loads with Ocean Sapphire theme fully rendered in under 3 seconds on standard broadband connection
- **SC-002**: Lighthouse Accessibility score maintains ≥95 after Ocean Sapphire implementation
- **SC-003**: All text contrast ratios meet WCAG AAA standards (≥7:1 for normal text, verified by automated contrast checker)
- **SC-004**: Shimmer animations on stats cards and chapter cards achieve 60fps on mid-range devices (tested on devices with equivalent specs to iPhone 12 and Galaxy S21)
- **SC-005**: Docusaurus build completes successfully with no CSS errors or console warnings related to Ocean Sapphire styling
- **SC-006**: All 21 existing textbook chapters render correctly with Ocean Sapphire theme without broken layouts
- **SC-007**: Three.js landing page animation maintains 60fps frame rate without janky performance
- **SC-008**: Users with `prefers-reduced-motion` enabled see static or simplified animations that don't trigger motion sickness
- **SC-009**: Mobile responsive breakpoints (390px, 360px) display Ocean Sapphire theme correctly with all interactive elements accessible
- **SC-010**: Browser fallbacks work correctly in browsers without `backdrop-filter` support (Firefox pre-103, Safari pre-15.4), providing readable alternative styling
- **SC-011**: Visual regression test suite passes with screenshot comparison for landing page and 3 sample chapter pages, and automated CSS validation confirms all Ocean Sapphire CSS variables are defined and contrast ratios meet WCAG AAA standards

## Assumptions

1. **Docusaurus 3.x CSS Architecture**: The existing Docusaurus installation uses CSS Modules or global CSS that can be extended with Ocean Sapphire variables. We assume `custom.css` in `website/src/css/` is the appropriate location for theme overrides.

2. **Landing Page Creation**: This feature includes creating a new custom landing page from scratch at `website/src/pages/index.tsx` (React/TypeScript component). The landing page will be built with Ocean Sapphire design system integrated from the start, including markup structure, Three.js scene integration, and all Ocean Sapphire styling.

3. **Three.js Already Integrated**: We assume Three.js library is already installed and a basic animated scene exists on the landing page. If not, Three.js installation and scene setup are additional prerequisites.

4. **Modern Browser Target**: We assume target browsers support CSS variables (96%+ global support), CSS Grid/Flexbox (98%+ support), and GPU-accelerated animations. Browsers older than IE 11 are explicitly not supported.

5. **No Breaking Changes to Content**: We assume Ocean Sapphire styling is purely additive and doesn't require changes to existing markdown chapter content. All 21 chapters should render correctly with only CSS updates.

6. **Accessibility is Non-Negotiable**: Per Constitution v1.1.0, we assume WCAG AAA compliance is mandatory. Any design choice that violates accessibility standards must be modified or removed.

7. **Performance Budget**: We assume a performance budget of <3 seconds for landing page load (as specified in Constitution). Glassmorphism effects and animations must not exceed this budget.

8. **Typography Licensing**: We assume Georgia font (system font) and Fira Code (Google Fonts or similar) have appropriate licensing for educational use. No custom font purchases are required.

9. **Mobile-First Responsive Design**: We assume all Ocean Sapphire components are designed mobile-first with progressive enhancement for larger screens. Touch interactions work equivalently to hover states.

10. **Dark Mode Only (Initial Scope)**: We assume Ocean Sapphire implementation focuses on dark mode styling as specified in Constitution v1.1.0. Light mode support can be added later as an enhancement using the CSS variable system.

## Dependencies

- **Docusaurus 3.x**: Static site generator and build system. Ocean Sapphire theme relies on Docusaurus CSS customization capabilities and plugin architecture.
- **Three.js**: JavaScript 3D library for landing page animated scene with blue gradient edges. Version compatibility with Docusaurus required.
- **CSS Custom Properties (CSS Variables)**: Modern CSS feature for centralized theme management. Requires browser support for `:root` variables and `var()` function.
- **backdrop-filter CSS Property**: Required for glassmorphism effects. Supported in Chrome 76+, Firefox 103+, Safari 15.4+, Edge 79+. Fallback styling needed for older browsers.
- **prefers-reduced-motion Media Query**: Accessibility feature for respecting user motion preferences. Supported in modern browsers (Chrome 74+, Firefox 63+, Safari 10.1+).
- **Constitution v1.1.0**: Governance document defining Ocean Sapphire design system specifications. All colors, typography, and effects must match constitutional requirements.
- **Existing Textbook Content (002-textbook)**: 21 chapters in markdown format. Ocean Sapphire theme must style these chapters without requiring content changes.
- **Visual Regression Testing Tools**: Screenshot comparison tools (e.g., Percy, Chromatic, or Playwright visual comparisons) for automated validation of Ocean Sapphire rendering across pages and browsers.
- **CSS Validation Tools**: Automated contrast ratio checkers and CSS variable validators to ensure WCAG AAA compliance and proper variable usage in compiled stylesheets.

## Out of Scope

1. **Light Mode Theme**: Ocean Sapphire implementation focuses exclusively on dark mode. Light mode variant is not included in this feature scope.
2. **Component Library Refactoring**: We're applying Ocean Sapphire to existing components, not rebuilding component architecture. Major React component refactoring is excluded.
3. **Advanced Three.js Scenes**: Landing page animation updates are limited to color/gradient changes. Complex new 3D models, particle systems, or advanced WebGL effects are out of scope.
4. **Interactive Theme Switcher**: No user-facing UI for toggling between old theme and Ocean Sapphire. Ocean Sapphire becomes the default and only theme.
5. **Animated Page Transitions Between Routes**: Smooth fade-in within a page is included. Complex route transition animations (e.g., page slide/flip between chapters) are out of scope.
6. **Custom Icon Set**: We're using existing icons (💎 for highlight boxes). Creating a custom icon library with Ocean Sapphire styling is excluded.
7. **Backend/API Changes**: Ocean Sapphire is purely a frontend/CSS feature. No server-side, database, or API modifications are required or included.
8. **User Personalization**: No user preferences for theme customization (e.g., "adjust blue intensity"). Ocean Sapphire applies uniformly to all users.
9. **A/B Testing Infrastructure**: No experimental framework for testing Ocean Sapphire against the old theme. Implementation is final and non-reversible.
10. **Feature Flags or Gradual Rollout**: Ocean Sapphire deploys as immediate full replacement with no feature flags, phased user rollout, or opt-in beta period. All users see Ocean Sapphire simultaneously upon deployment.
11. **Documentation Website for Ocean Sapphire**: We're applying the theme to the textbook, not creating separate style guide documentation. Design system docs are out of scope.

## Notes

- **Constitution Alignment**: This feature directly implements the Ocean Sapphire Design Philosophy added to Constitution v1.1.0. All specifications (colors, typography, animations) are derived from constitutional requirements to ensure governance compliance.
- **Accessibility Priority**: WCAG AAA compliance is mandatory per Constitution. Contrast ratios (`#b8d4ff` on `#001529` = 8.2:1) have been validated against WCAG standards. All animations respect `prefers-reduced-motion`.
- **Performance Considerations**: Glassmorphism (`backdrop-filter: blur(10px)`) is GPU-accelerated but can impact performance on low-end devices. Fallback styling uses semi-transparent solid backgrounds for browsers without support.
- **Incremental Implementation by User Story**: User Stories are prioritized (P1→P2→P3→P4) to enable incremental *implementation* phases during development. However, deployment is immediate full replacement—all completed user stories deploy together as the default theme for all users simultaneously (no gradual user rollout or feature flags).
- **CSS Variable Strategy**: Centralizing Ocean Sapphire colors in CSS variables (`:root`) enables future theme iterations without touching component code. Variables should follow naming convention `--ocean-[category]-[variant]`. All CSS variable usage must include inline fallback pattern (e.g., `color: #b8d4ff; color: var(--ocean-text-pale);`) for graceful degradation in older browsers.
- **Typography Performance**: Georgia is a system font (no download required). Fira Code uses `font-display: swap` for loading optimization and font stack `'Fira Code', 'Courier New', Consolas, monospace` ensures automatic fallback to system monospace fonts if CDN fails or loads slowly, preventing FOUT (Flash of Unstyled Text) and maintaining code readability.
- **Browser Compatibility**: Target browsers are Chrome/Edge 90+, Firefox 88+, Safari 14+. Older browsers receive graceful degradation: inline CSS fallback values ensure Ocean Sapphire colors render correctly even without CSS variable support, no glassmorphism effects (fallback to semi-transparent backgrounds), and simplified animations. All browsers remain functional with readable content.
- **Testing Strategy**: Visual regression testing with automated screenshot comparison is required for Ocean Sapphire validation. The test suite must capture landing page and 3 sample chapter pages, plus automated CSS validation to verify all variables are defined and contrast ratios meet WCAG AAA. This ensures consistent rendering and catches unintended style changes in CI/CD pipelines.
- **Collaboration with Textbook Content**: This feature works alongside existing 002-textbook content. No markdown changes required, but authors should preview chapters with Ocean Sapphire to verify code block contrast and heading hierarchy.
- **Future Enhancements**: Light mode variant, user customization, and theme switcher UI are potential P5+ stories for future iterations if user feedback indicates demand.
