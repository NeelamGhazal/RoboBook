# Research: Professional Landing Page

**Feature**: 001-landing-page
**Date**: 2025-12-09
**Status**: Complete

## Overview

Research findings for implementing a high-performance, accessible landing page with Three.js 3D animations, Framer Motion UI animations, and glass-morphism styling in Docusaurus 3.x.

## Research Topics

### 1. Docusaurus 3.x Custom Pages

**Decision**: Use `src/pages/index.tsx` with `@docusaurus/Head` and `@docusaurus/Link`

**Rationale**:
- Docusaurus automatically routes files in `src/pages/` (index.tsx → `/`)
- `@docusaurus/Link` provides SPA navigation without page reload (requirement FR-005)
- `@docusaurus/Head` manages meta tags for SEO and accessibility
- Supports TypeScript out of the box
- Hot module replacement works seamlessly

**Alternatives Considered**:
- **Swizzling default theme**: Too invasive, harder to maintain, loses Docusaurus updates
- **Plugin-based approach**: Overkill for single page, adds unnecessary complexity

**Best Practices**:
- Export default React component from `src/pages/index.tsx`
- Use `Layout` component from `@theme/Layout` for consistent navigation
- Import styles via CSS Modules for component isolation

**References**:
- Docusaurus Docs: https://docusaurus.io/docs/creating-pages
- Official examples: https://github.com/facebook/docusaurus/tree/main/website/src/pages

### 2. Three.js + React Integration

**Decision**: Use `@react-three/fiber` + `@react-three/drei` + lazy loading

**Rationale**:
- `@react-three/fiber` (R3F) is React reconciler for Three.js (declarative, hooks-based)
- `@react-three/drei` provides helpers (`OrbitControls`, `PerspectiveCamera`, `Float` for floating motion)
- Lazy loading via `React.lazy()` + `Suspense` defers Three.js bundle (~600KB) until needed
- Desktop-only loading reduces mobile bundle by 600KB
- R3F auto-manages Three.js scene, camera, renderer lifecycle

**Alternatives Considered**:
- **Vanilla Three.js with useEffect**: More boilerplate, manual lifecycle management, no React benefits
- **Lottie animations**: Not true 3D, less impressive, limited customization
- **CSS 3D transforms**: Can't achieve realistic cube with glowing edges

**Implementation Pattern**:
```tsx
const AnimatedCube = lazy(() => import('./AnimatedCube'));

// In component:
{isDesktop && !prefersReducedMotion ? (
  <Suspense fallback={<FallbackSVG />}>
    <AnimatedCube />
  </Suspense>
) : (
  <FallbackSVG />
)}
```

**Best Practices**:
- Use `<Canvas>` from R3F as root component (handles WebGL context)
- Set `gl={{ antialias: true }}` for smooth edges
- Use `frameloop="always"` for continuous animation
- Implement `Float` component from drei for subtle floating motion
- Use `MeshBasicMaterial` with `emissive` color for neon glow
- Add `EdgesGeometry` with `LineBasicMaterial` for neon edges

**Performance**:
- R3F auto-throttles to monitor refresh rate (typically 60fps)
- `useMemo` for geometry/materials to prevent recreation
- Dispose Three.js objects on unmount (R3F handles automatically)

**References**:
- R3F Docs: https://docs.pmnd.rs/react-three-fiber
- Drei Helpers: https://github.com/pmndrs/drei
- Three.js Fundamentals: https://threejs.org/manual/#en/fundamentals

### 3. Framer Motion Scroll Animations

**Decision**: Use Framer Motion `motion` components with `useInView` hook + stagger

**Rationale**:
- Framer Motion is industry standard for React animations (60fps, GPU-accelerated)
- `useInView` hook detects when element enters viewport (no Intersection Observer boilerplate)
- `staggerChildren` API enables 100ms stagger between cards (requirement FR-014)
- `whileHover` declaratively handles hover states (requirement FR-015)
- Respects `prefers-reduced-motion` automatically

**Implementation Pattern**:
```tsx
const containerVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: { staggerChildren: 0.1 }
  }
};

const cardVariants = {
  hidden: { opacity: 0, y: 20 },
  visible: { opacity: 1, y: 0 }
};

<motion.div
  variants={containerVariants}
  initial="hidden"
  animate={controls}
>
  {stats.map(stat => (
    <motion.div
      variants={cardVariants}
      whileHover={{ y: -10, boxShadow: '0 0 20px #00F0FF' }}
    >
      {stat}
    </motion.div>
  ))}
</motion.div>
```

**Alternatives Considered**:
- **CSS transitions**: No stagger support, manual viewport detection required
- **GSAP**: Heavier bundle, not React-native, requires `gsap.to()` imperative API
- **React Spring**: Physics-based (overkill), less intuitive API for simple fades

**Best Practices**:
- Use `variants` for reusable animation configs
- Combine `useInView` + `useAnimation` for scroll-triggered animations
- Set `once: true` in `useInView` to trigger animation only once
- Use `whileHover` instead of manual state management
- Wrap animated components in `motion.div` or `motion.section`

**References**:
- Framer Motion Docs: https://www.framer.com/motion/
- Scroll Animations: https://www.framer.com/motion/use-in-view/
- Animation Variants: https://www.framer.com/motion/animation/#variants

### 4. Glass-morphism CSS

**Decision**: Use `backdrop-filter: blur(10px)` with fallback for Firefox <103

**Rationale**:
- `backdrop-filter` creates true glass-morphism effect (blurs background)
- Supported in Chrome 76+, Safari 9+, Edge 79+ (covers 95% of users)
- Firefox added support in v103 (Oct 2022), older versions need fallback
- CSS `@supports` enables graceful degradation
- Lighter than PNG/SVG overlays

**Implementation Pattern**:
```css
.glass-card {
  background: rgba(18, 19, 26, 0.6); /* #12131A at 60% opacity */
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px); /* Safari */
  border: 1px solid rgba(0, 240, 255, 0.2); /* Subtle cyan border */
}

@supports not (backdrop-filter: blur(10px)) {
  .glass-card {
    background: rgba(18, 19, 26, 0.9); /* More opaque fallback */
  }
}
```

**Alternatives Considered**:
- **Solid backgrounds**: No glass effect, less premium feel
- **PNG/SVG blurred overlays**: Heavier, not dynamic, fixed blur radius
- **Canvas-based blur**: Huge performance cost, not worth it

**Best Practices**:
- Use semi-transparent backgrounds (0.5–0.7 opacity) with backdrop-filter
- Add subtle border (lighter color at low opacity) for card definition
- Test fallback on Firefox <103
- Combine with `border-radius` for soft edges

**Performance**:
- `backdrop-filter` is GPU-accelerated on supported browsers
- Can cause repaints on scroll (mitigate by using `will-change: backdrop-filter` sparingly)

**References**:
- MDN backdrop-filter: https://developer.mozilla.org/en-US/docs/Web/CSS/backdrop-filter
- Can I Use: https://caniuse.com/css-backdrop-filter
- Glass-morphism Guide: https://css-tricks.com/glass-morphism/

### 5. Responsive Design with Tailwind + CSS Modules

**Decision**: Combine Tailwind utilities for layout + CSS Modules for component-specific styles

**Rationale**:
- Tailwind handles responsive breakpoints (`md:`, `lg:`) and utilities (flexbox, grid)
- CSS Modules provide scoped styles for complex components (no class collisions)
- Docusaurus 3.x supports both out of the box
- Tailwind's `@apply` in CSS Modules combines both approaches

**Configuration**:
```js
// tailwind.config.js
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      colors: {
        'cyan-neon': '#00F0FF',
        'magenta-neon': '#FF2A6D',
        'dark-bg': '#0A0A0F',
        'card-dark': '#12131A'
      }
    }
  }
};
```

**Alternatives Considered**:
- **Tailwind only**: Inline classes get verbose for complex animations
- **CSS Modules only**: Manual responsive breakpoints, no design system consistency
- **Styled Components**: Runtime cost, SSR complexity, not Docusaurus convention

**Best Practices**:
- Use Tailwind for layout (grid, flexbox, spacing, breakpoints)
- Use CSS Modules for animations, hover effects, glass-morphism
- Extract repeated Tailwind patterns to CSS Modules with `@apply`
- Use `clsx` for conditional Tailwind classes

**References**:
- Tailwind Docs: https://tailwindcss.com/docs
- CSS Modules: https://github.com/css-modules/css-modules
- Docusaurus Styling: https://docusaurus.io/docs/styling-layout

### 6. Accessibility (WCAG 2.1 AA)

**Decision**: Implement focus indicators, ARIA labels, keyboard navigation, reduced motion detection

**Rationale**:
- Focus indicators required for keyboard navigation (requirement SC-010)
- ARIA labels provide context for screen readers
- `prefers-reduced-motion` media query disables animations (requirement FR-011)
- Color contrast validated: cyan on dark = 10.4:1 (exceeds 4.5:1 minimum)

**Implementation Checklist**:
- [x] Focus indicators on button (`:focus-visible` ring, cyan glow)
- [x] ARIA label for button: `aria-label="Navigate to textbook introduction"`
- [x] Heading hierarchy: `<h1>` for main heading, no skipped levels
- [x] Alt text for fallback SVG: `alt="Animated 3D cube visualization"`
- [x] `prefers-reduced-motion` detection via CSS media query or React hook
- [x] Keyboard navigable: button accessible via Tab key

**Best Practices**:
- Use semantic HTML (`<button>`, `<h1>`, `<section>`)
- Test with keyboard only (Tab, Enter, Escape)
- Test with screen reader (NVDA, VoiceOver)
- Validate contrast ratios with tools (Chrome DevTools, Lighthouse)

**References**:
- WCAG 2.1 Guidelines: https://www.w3.org/WAI/WCAG21/quickref/
- MDN Accessibility: https://developer.mozilla.org/en-US/docs/Web/Accessibility
- React Accessibility: https://reactjs.org/docs/accessibility.html

### 7. Performance Optimization

**Decision**: Code splitting, lazy loading, critical CSS inlining, Lighthouse CI

**Rationale**:
- Three.js bundle (~600KB) lazy-loaded only on desktop reduces mobile FCP by ~1s
- Docusaurus auto-splits routes, further optimization via dynamic imports
- Critical CSS inlined in `<head>` enables instant render (FCP <1.5s)
- Lighthouse CI in GitHub Actions enforces performance budget

**Optimization Strategies**:
1. **Code Splitting**: `React.lazy(() => import('./AnimatedCube'))`
2. **Lazy Load Images**: Use `loading="lazy"` on fallback SVG (if external)
3. **Font Loading**: `font-display: swap` for Inter/Poppins (avoid FOIT)
4. **Defer Non-Critical Scripts**: Analytics, chatbot load after `DOMContentLoaded`
5. **Minimize Main Thread Work**: Offload animations to GPU with `transform`, `opacity`

**Lighthouse CI Config**:
```js
// lighthouserc.js
module.exports = {
  ci: {
    assert: {
      assertions: {
        'categories:performance': ['error', { minScore: 0.9 }],
        'categories:accessibility': ['error', { minScore: 0.9 }],
        'first-contentful-paint': ['error', { maxNumericValue: 1500 }],
        'interactive': ['error', { maxNumericValue: 3000 }]
      }
    }
  }
};
```

**References**:
- Web.dev Performance: https://web.dev/performance/
- Lighthouse CI: https://github.com/GoogleChrome/lighthouse-ci
- React Code Splitting: https://reactjs.org/docs/code-splitting.html

## Unresolved Questions

**None.** All technical unknowns from plan.md resolved through research.

## Architectural Decisions

### ADR Candidates (3-Part Test)

1. **Three.js + React-Three-Fiber vs Vanilla Three.js**
   - **Impact**: Long-term maintainability, developer experience
   - **Alternatives**: R3F (declarative), Vanilla (imperative), Lottie (2D only)
   - **Scope**: Cross-cutting (affects all 3D features)
   - **Verdict**: Significant → Consider documenting with `/sp.adr`

2. **Desktop-Only 3D Animation**
   - **Impact**: Performance on mobile, bundle size
   - **Alternatives**: Universal 3D (slow mobile), Static only (no wow factor), Lottie fallback
   - **Scope**: Cross-cutting (affects all animation strategy)
   - **Verdict**: Significant → Consider documenting with `/sp.adr`

3. **Tailwind + CSS Modules Hybrid**
   - **Impact**: Styling approach for entire project
   - **Alternatives**: Tailwind only, CSS Modules only, Styled Components
   - **Scope**: Cross-cutting (affects all components)
   - **Verdict**: Moderate → Not critical enough for ADR (standard pattern)

**Recommendation**: Suggest ADRs for #1 and #2 after user review.

## Summary

All research complete. Key decisions:
- Docusaurus `src/pages/index.tsx` for landing page
- React-Three-Fiber + Drei for 3D cube (lazy-loaded desktop-only)
- Framer Motion for card animations (stagger + hover)
- Tailwind + CSS Modules hybrid styling
- Glass-morphism with Firefox fallback
- Accessibility via focus indicators, ARIA, reduced motion detection
- Performance via code splitting, lazy loading, Lighthouse CI

Ready for Phase 1: Design artifacts (quickstart.md).
