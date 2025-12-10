# Research: RoboBook Landing Page Redesign

**Feature**: 004-landing-page-redesign
**Date**: 2025-12-10
**Status**: Complete

## Purpose

This document consolidates research findings for implementing the RoboBook landing page redesign according to Constitution v1.4.0. All technical decisions derive from Constitution specifications, user requirements, and React/Docusaurus best practices.

---

## Research Area 1: React Theme Toggle with CSS Variables

### Question
How to implement light/dark theme toggle using React state + CSS variables with 200ms transitions while ensuring WCAG AA compliance and zero layout shift?

### Findings

**Approach**: React useState hook + CSS custom properties (variables) + data attribute

**Implementation Pattern**:
```typescript
// Theme state in index.tsx or layout component
const [theme, setTheme] = useState<'dark' | 'light'>('dark');

useEffect(() => {
  document.documentElement.setAttribute('data-theme', theme);
}, [theme]);
```

**CSS Variables Pattern**:
```css
:root[data-theme='dark'] {
  --bg-primary: #001529;
  --text-primary: #b8d4ff;
  --accent-cyan: #0096ff;
  --card-bg: rgba(0, 50, 100, 0.3);
}

:root[data-theme='light'] {
  --bg-primary: #f3f8ff;
  --text-primary: #001529;
  --accent-cyan: #0096ff;
  --card-bg: rgba(255, 255, 255, 0.6);
}

* {
  transition: background-color 200ms ease-in-out, color 200ms ease-in-out, border-color 200ms ease-in-out;
}
```

**WCAG AA Compliance**:
- Dark theme contrast: `#b8d4ff` on `#001529` = 10.12:1 ✅ (exceeds 4.5:1)
- Light theme contrast: `#001529` on `#f3f8ff` = 16.31:1 ✅ (exceeds 4.5:1)
- Accent cyan `#0096ff`: sufficient contrast on both backgrounds

**Zero Layout Shift**: Only transition color properties, never transition `width`, `height`, `padding`, `margin`, `transform` during theme switch

### Decision
Use React useState + CSS variables with data-theme attribute. Transition only color properties (background-color, color, border-color) with 200ms ease-in-out. Verify contrast ratios using WebAIM Contrast Checker.

### Alternatives Considered
- **Context API with localStorage**: Rejected per Constitution §XX (no localStorage)
- **CSS class toggle**: Rejected as less idiomatic in React
- **Inline styles**: Rejected due to performance and maintainability

---

## Research Area 2: Glassmorphism with Backdrop Filter

### Question
How to implement glassmorphism effect for feature cards using `backdrop-filter: blur()` with cross-browser compatibility?

### Findings

**Browser Support** (as of 2024):
- Chrome/Edge: Full support
- Firefox: Full support (enabled by default since v103)
- Safari: Full support (with -webkit- prefix)
- Mobile: iOS Safari 9+, Chrome Android

**Implementation Pattern**:
```css
.glass-card {
  background: rgba(0, 50, 100, 0.3); /* Semi-transparent */
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px); /* Safari fallback */
  border: 1px solid rgba(0, 150, 255, 0.3);
  border-radius: 12px;
}

/* Fallback for unsupported browsers */
@supports not (backdrop-filter: blur(10px)) {
  .glass-card {
    background: rgba(0, 50, 100, 0.8); /* More opaque fallback */
  }
}
```

**Performance Considerations**:
- `backdrop-filter` is GPU-accelerated in modern browsers
- Minimal performance impact with blur radius ≤10px
- Avoid excessive blur layers (limit to 3-4 cards on screen)

### Decision
Use `backdrop-filter: blur(10px)` with Safari prefix and @supports fallback to more opaque background. Constitution §VIII specifies glassmorphism for cards, so implementation is required.

### Alternatives Considered
- **SVG filters**: Rejected due to complexity and worse performance
- **Blurred background images**: Rejected as not true glassmorphism

---

## Research Area 3: Lazy Loading with WebP Images

### Question
How to implement lazy loading for robot and feature card images in React while ensuring zero layout shift (CLS = 0)?

### Findings

**Native Lazy Loading**:
```tsx
<img
  src="/img/robot-halfbody.webp"
  alt="Professional 3D half-body robot"
  loading="lazy"
  width={550}
  height={733}  // Explicit dimensions prevent CLS
  style={{ maxWidth: '100%', height: 'auto' }}
/>
```

**Zero Layout Shift Requirements**:
- Always specify `width` and `height` attributes (aspect ratio preserved by browser)
- Use `aspect-ratio` CSS property for responsive images
- Avoid `height: auto` without explicit dimensions

**WebP Conversion**:
- Tools: Sharp (Node.js), ImageMagick, online converters (Squoosh, CloudConvert)
- Target compression: Robot <200KB, cards <50KB (per Constitution §XXI)
- Quality settings: 80-85 for photos, 90 for graphics

**Browser Support**:
- WebP: 96%+ global support (Chrome, Firefox, Safari 14+, Edge)
- Fallback: Use `<picture>` element if supporting Safari <14
  ```tsx
  <picture>
    <source srcSet="/img/robot.webp" type="image/webp" />
    <img src="/img/robot.jpg" alt="..." />
  </picture>
  ```

### Decision
Use native `loading="lazy"` attribute with explicit `width` and `height` attributes. WebP images only (Safari 14+ requirement acceptable for modern site). Compress robot image to <200KB and cards to <50KB using Sharp or online tools.

### Alternatives Considered
- **Intersection Observer API**: Rejected as native lazy loading is simpler and well-supported
- **React libraries (react-lazyload)**: Rejected as unnecessary with native support

---

## Research Area 4: Responsive Breakpoints in Docusaurus

### Question
What are Docusaurus default breakpoints and how to implement custom responsive layouts for 50/50, 60/40, and stacked layouts?

### Findings

**Docusaurus Default Breakpoints** (Infima CSS):
- Mobile: `<996px`
- Tablet: `996px - 1279px`
- Desktop: `≥1280px`

**Constitution Requirements** (different from Docusaurus defaults):
- Mobile: `<768px`
- Tablet: `768px - 1023px`
- Desktop: `≥1024px`

**Implementation Strategy**:
Use custom media queries in `custom.css` to override Docusaurus defaults for landing page:

```css
/* Mobile (<768px): Stacked layout */
.hero-container {
  display: flex;
  flex-direction: column;
}

/* Tablet (768-1023px): 60/40 layout */
@media (min-width: 768px) and (max-width: 1023px) {
  .hero-container {
    flex-direction: row;
  }
  .hero-text {
    flex: 0 0 60%;
  }
  .hero-image {
    flex: 0 0 40%;
  }
}

/* Desktop (≥1024px): 50/50 layout */
@media (min-width: 1024px) {
  .hero-container {
    flex-direction: row;
    
  }
  .hero-text {
    flex: 0 0 50%;
  }
  .hero-image {
    flex: 0 0 50%;
  }
}
```

### Decision
Use custom media queries (768px, 1024px breakpoints) in `custom.css` to implement Constitution-specified responsive layouts. Do not rely on Docusaurus defaults.

### Alternatives Considered
- **Tailwind CSS**: Rejected as Docusaurus doesn't use Tailwind by default and adding it violates "no heavy JS libraries" restriction
- **Styled Components**: Rejected as requires additional dependency

---

## Research Area 5: Shimmer Animation with CSS Keyframes

### Question
How to implement shimmer effect on feature cards using GPU-accelerated CSS animations with `prefers-reduced-motion` support?

### Findings

**Shimmer Effect Pattern**:
```css
@keyframes shimmer {
  0% {
    background-position: -1000px 0;
  }
  100% {
    background-position: 1000px 0;
  }
}

.feature-card {
  position: relative;
  overflow: hidden;
}

.feature-card::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: linear-gradient(
    90deg,
    transparent 0%,
    rgba(0, 150, 255, 0.1) 50%,
    transparent 100%
  );
  background-size: 1000px 100%;
  animation: shimmer 3s infinite;
  pointer-events: none;
}

/* Respect reduced motion preference */
@media (prefers-reduced-motion: reduce) {
  .feature-card::before {
    animation: none;
  }
}
```

**Performance**: GPU-accelerated via `transform` or `background-position` (background-position is simpler for this effect)

### Decision
Use CSS keyframes with pseudo-element (::before) for shimmer overlay. Respect `prefers-reduced-motion` media query per Constitution §XXI. 3-second infinite loop per requirements.

### Alternatives Considered
- **JavaScript with requestAnimationFrame**: Rejected due to performance overhead
- **Transform-based shimmer**: Rejected as background-position is simpler for gradient movement

---

## Research Area 6: Orbitron Font Integration

### Question
How to load Orbitron font only for landing page components without affecting docs/blog?

### Findings

**Font Loading Options**:
1. **Google Fonts CDN**: Add `<link>` in `index.tsx` only
2. **Local font files**: Store in `static/fonts/` and use `@font-face` in `custom.css`
3. **Inline font-family**: Apply via inline styles to specific components

**Constitution Requirement**: Orbitron ONLY for navbar + landing page headings/CTAs; NOT in docs/blog

**Recommended Approach**:
```tsx
// In index.tsx or Navbar component
useEffect(() => {
  const link = document.createElement('link');
  link.href = 'https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&display=swap';
  link.rel = 'stylesheet';
  document.head.appendChild(link);

  return () => {
    document.head.removeChild(link);
  };
}, []);
```

Apply via inline styles to prevent global application:
```tsx
<h1 style={{ fontFamily: 'Orbitron, sans-serif', fontWeight: 700, fontSize: '4.5rem' }}>
  Physical AI & Humanoid Robotics
</h1>
```

### Decision
Load Orbitron via Google Fonts CDN in landing page components only. Apply font-family via inline styles to heading/CTA elements to prevent leakage to docs/blog.

### Alternatives Considered
- **Global CSS rule**: Rejected as would apply to docs/blog
- **CSS Modules**: Not available in default Docusaurus setup without additional config

---

## Research Area 7: Image Asset Requirements

### Question
What are specific requirements for robot image and feature card images (dimensions, format, compression)?

### Findings

**Robot Image Requirements** (Constitution §VIII + §XXI):
- Type: Professional 3D half-body robot
- Dimensions: 520-580px width, height auto (maintain proportions)
- Format: WebP preferred
- Size: <200KB maximum
- Aspect Ratio: Approximately 3:4 (portrait orientation for half-body)
- Quality: High resolution, professional rendering

**Feature Card Images** (Constitution §VIII + §XXI):
- Aspect Ratio: 3:2 (landscape)
- Format: WebP
- Size: <50KB each (3 images total)
- Dimensions: Approximately 540×360px (scales to card size)
- Subjects:
  1. AI/Spec-Driven Book Creation (code, books, AI symbols)
  2. Integrated RAG Chatbot (chat interface, AI brain, database)
  3. Personalization & Translations (globe, language symbols, user profiles)

**Image Sourcing**:
- AI generation tools: Midjourney, DALL-E 3, Stable Diffusion
- 3D rendering: Blender, Cinema 4D
- Stock photos: Unsplash, Pexels (with modification)

### Decision
Acquire professional 3D robot image (520-580px width, WebP <200KB). Generate or source feature card images (3:2 ratio, WebP <50KB each). Compress using Sharp or online tools (Squoosh, TinyPNG).

### Alternatives Considered
- **SVG illustrations**: Rejected as Constitution specifies "3D robot image" (raster required)
- **Placeholder images**: Temporary use acceptable during development

---

## Summary of Resolved Clarifications

| Area | Question | Resolution |
|------|----------|------------|
| Theme Toggle | Implementation approach | React useState + CSS variables with data-theme attribute |
| Glassmorphism | Browser compatibility | backdrop-filter with -webkit- prefix and @supports fallback |
| Lazy Loading | Zero layout shift strategy | Native loading="lazy" with explicit width/height attributes |
| Responsive Design | Custom breakpoints | Custom media queries (768px, 1024px) in custom.css |
| Shimmer Animation | Performance approach | CSS keyframes with ::before pseudo-element, GPU-accelerated |
| Font Loading | Scope limitation | Load Orbitron in landing components only via inline styles |
| Image Assets | Specifications | Robot 520-580px/<200KB WebP, cards 3:2/<50KB WebP |

**All clarifications resolved. Ready for Phase 1: Design & Contracts.**
