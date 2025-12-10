# Research: Ocean Sapphire Design System — Docusaurus Theme Implementation

**Feature**: 003-ocean-sapphire-theme | **Date**: 2025-12-10
**Purpose**: Document technical decisions, browser compatibility strategies, and implementation patterns for Ocean Sapphire design system

---

## Decision 1: Three.js Integration with Docusaurus (SSR-Safe Pattern)

**Decision**: Use Docusaurus `<BrowserOnly>` component to wrap Three.js scenes, preventing Server-Side Rendering conflicts

**Rationale**:
- **SSR Compatibility**: Three.js requires browser APIs (`window`, `document`, WebGL) that don't exist during Docusaurus static site generation. The `<BrowserOnly>` component ensures Three.js code only executes in the browser after React hydration.
- **Build Stability**: Without this pattern, Docusaurus builds fail with "ReferenceError: window is not defined" during Node.js SSR phase.
- **Official Pattern**: This is Docusaurus's recommended approach for integrating browser-only libraries, documented in official Docusaurus Client API.

**Alternatives Considered**:
1. **Conditional rendering with `typeof window !== 'undefined'`** - Rejected because React will produce mismatched DOM between server and client, causing hydration errors and potential UI corruption.
2. **Dynamic imports with `next/dynamic` (Next.js pattern)** - Rejected because Docusaurus doesn't support Next.js-specific APIs; `<BrowserOnly>` is the Docusaurus equivalent.
3. **React Three Fiber with Suspense** - Rejected because ReactDOMServer doesn't yet support Suspense during SSR, causing build failures.

**Implementation Pattern**:

```typescript
// website/src/pages/index.tsx
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function LandingPage() {
  return (
    <div className="ocean-landing">
      <BrowserOnly fallback={<div>Loading 3D scene...</div>}>
        {() => {
          // Import Three.js only in browser context
          const ThreeScene = require('../components/ThreeScene').default;
          return <ThreeScene />;
        }}
      </BrowserOnly>
    </div>
  );
}
```

```typescript
// website/src/components/ThreeScene.tsx
import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

export default function ThreeScene() {
  const mountRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!mountRef.current) return;

    // Three.js scene setup (safe because this only runs in browser)
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });

    renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(renderer.domElement);

    // Ocean Sapphire gradient edges (#0096ff → #00d4ff)
    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshBasicMaterial({
      color: 0x0096ff,
      wireframe: true
    });
    const cube = new THREE.Mesh(geometry, material);
    scene.add(cube);

    camera.position.z = 5;

    const animate = () => {
      requestAnimationFrame(animate);
      cube.rotation.x += 0.01;
      cube.rotation.y += 0.01;
      renderer.render(scene, camera);
    };
    animate();

    // Cleanup
    return () => {
      renderer.dispose();
      mountRef.current?.removeChild(renderer.domElement);
    };
  }, []);

  return <div ref={mountRef} style={{ width: '100%', height: '400px' }} />;
}
```

**Best Practices**:
- **Fallback Content**: Always provide meaningful fallback for SSR phase (loading message or static image)
- **Lazy Import**: Use `require()` inside the BrowserOnly callback to avoid importing Three.js during SSR
- **Cleanup**: Implement proper Three.js disposal in `useEffect` cleanup to prevent memory leaks
- **Performance**: Use `requestAnimationFrame` for 60fps animations; target Ocean Sapphire gradient colors (#0096ff → #00d4ff)

**Sources**:
- [Docusaurus Client API - BrowserOnly](https://docusaurus.io/docs/docusaurus-core/)
- [Building Interactive Documentation with Docusaurus BrowserOnly](https://developers.onwardplatforms.com/blog/using-browseronly-in-docusaurus)
- [Static site generation (SSG) | Docusaurus](https://docusaurus.io/docs/next/advanced/ssg)

---

## Decision 2: Glassmorphism Browser Compatibility (Fallback Strategy)

**Decision**: Use `@supports` feature query to provide solid semi-transparent backgrounds for browsers without `backdrop-filter` support

**Rationale**:
- **Progressive Enhancement**: Modern browsers (Chrome 76+, Firefox 103+, Safari 15.4+, Edge 79+) get glassmorphism effects; older browsers get readable fallback styling without breaking layout.
- **Graceful Degradation**: Fallback uses higher opacity backgrounds (0.7-0.8 instead of 0.2-0.3) to ensure text readability when blur effect is unavailable.
- **Standards Compliance**: `@supports` is the W3C-recommended way to detect CSS feature support, avoiding fragile JavaScript-based detection.

**Alternatives Considered**:
1. **JavaScript feature detection with Modernizr** - Rejected because adds unnecessary JavaScript payload and complexity; CSS `@supports` is native and more performant.
2. **No fallback (require modern browsers only)** - Rejected because spec requires graceful degradation (FR-017) and Constitution mandates accessibility for wide audience.
3. **Polyfill for backdrop-filter** - Rejected because no reliable polyfill exists (backdrop-filter requires GPU acceleration that can't be emulated with JavaScript).

**Implementation Pattern**:

```css
/* website/src/css/custom.css */

/* Fallback for browsers without backdrop-filter support */
.ocean-card {
  background: rgba(0, 50, 100, 0.7); /* More opaque fallback */
  border: 1px solid rgba(0, 150, 255, 0.3);
  border-radius: 12px;
  padding: 2rem;
}

/* Enhanced glassmorphism for modern browsers */
@supports ((-webkit-backdrop-filter: blur(0px)) or (backdrop-filter: blur(0px))) {
  .ocean-card {
    background: rgba(0, 50, 100, 0.3); /* Less opaque with blur */
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px); /* Safari 15.4-17 requires prefix */
  }
}

/* Alternative pattern: Negative @supports for explicit fallback */
@supports not ((-webkit-backdrop-filter: none) or (backdrop-filter: none)) {
  .ocean-card {
    background: rgba(0, 50, 100, 0.8); /* Even more opaque for clarity */
  }
}
```

**Browser Support Matrix** (as of 2025):

| Browser | backdrop-filter Support | Fallback Needed |
|---------|------------------------|-----------------|
| Chrome 76+ | ✅ Native | No |
| Firefox 103+ | ✅ Native | No |
| Safari 15.4+ | ✅ Native (unprefixed in 17+) | No (use -webkit- prefix for 15.4-17) |
| Edge 79+ | ✅ Native | No |
| Firefox <103 | ❌ No support | Yes (solid background) |
| Safari <15.4 | ❌ No support | Yes (solid background) |

**Best Practices**:
- **Opacity Strategy**: Fallback backgrounds use 0.7-0.8 opacity; glassmorphism uses 0.2-0.3 opacity (blur compensates for transparency)
- **Vendor Prefixes**: Include `-webkit-backdrop-filter` for Safari 15.4-17 compatibility
- **Testing**: Verify readability in both modes using Firefox 88 (no backdrop-filter) and Chrome 120+ (with backdrop-filter)
- **Border Visibility**: Keep border `1px solid rgba(0, 150, 255, 0.3)` in both modes to maintain card structure

**Sources**:
- [backdrop-filter - CSS | MDN](https://developer.mozilla.org/en-US/docs/Web/CSS/backdrop-filter)
- [CSS Backdrop Filter | Can I use... Support tables](https://caniuse.com/css-backdrop-filter)
- [Backdrop Filter in CSS | RUSTCODE](https://www.rustcodeweb.com/2025/07/backdrop-filter-in-css.html)

---

## Decision 3: Visual Regression Testing (Playwright with GitHub Actions)

**Decision**: Use Playwright's built-in visual comparison with GitHub Actions for free-tier CI/CD, avoiding paid SaaS tools

**Rationale**:
- **Cost**: Playwright visual testing is completely free and open-source, running entirely in GitHub Actions free tier (2000 minutes/month for public repos). No external services or monthly fees.
- **Simplicity**: Playwright includes screenshot comparison out-of-the-box with `expect(page).toHaveScreenshot()`. No additional tooling or complex integrations required.
- **CI/CD Native**: GitHub Actions workflow auto-generated by Playwright CLI includes everything needed for automated testing on every PR.
- **Adequate for Use Case**: Ocean Sapphire needs validation that colors/gradients render correctly across pages. Playwright's pixel-perfect comparison meets this requirement without needing advanced features from Percy/Chromatic.

**Alternatives Considered**:
1. **Percy (by BrowserStack)** - Rejected because costs money after free tier (5000 snapshots, limited parallelization). Provides richer dashboards and cross-browser testing, but unnecessary overhead for Ocean Sapphire's focused needs.
2. **Chromatic** - Rejected because optimized for Storybook component testing (not full pages) and costs money. Best for design systems with component libraries, but Ocean Sapphire applies theme to existing Docusaurus pages.
3. **Manual visual inspection** - Rejected because human error prone and doesn't scale. Spec requires automated visual validation (SC-011).

**Why Playwright Over Percy/Chromatic for Ocean Sapphire**:
- **Ocean Sapphire Scope**: Testing full Docusaurus pages (landing page + 3 chapter samples) for CSS rendering, not isolated React components
- **Budget**: No budget allocated for paid testing tools; GitHub Actions free tier is sufficient
- **Team Size**: Solo developer or small team without need for collaborative visual review tools
- **Testing Focus**: Binary pass/fail for CSS colors/gradients, not collaborative design feedback

**Implementation Pattern**:

```typescript
// tests/visual-regression/ocean-sapphire.spec.ts
import { test, expect } from '@playwright/test';

test.describe('Ocean Sapphire Visual Regression', () => {
  test('Landing page renders with Ocean Sapphire theme', async ({ page }) => {
    await page.goto('/');

    // Wait for Three.js scene to load
    await page.waitForSelector('.ocean-landing', { state: 'visible' });

    // Full page screenshot
    await expect(page).toHaveScreenshot('landing-page.png', {
      fullPage: true,
      maxDiffPixels: 100, // Allow minor anti-aliasing differences
    });
  });

  test('Book reader page with Ocean Sapphire styling', async ({ page }) => {
    await page.goto('/docs/module-1-ros2/chapter-1-nodes-architecture');

    // Wait for content to load
    await page.waitForSelector('.theme-doc-markdown', { state: 'visible' });

    // TOC sidebar active state
    const tocActiveItem = page.locator('.table-of-contents__link--active');
    await expect(tocActiveItem).toHaveCSS('border-left', '3px solid rgb(0, 150, 255)');

    // Screenshot comparison
    await expect(page).toHaveScreenshot('chapter-page.png', {
      fullPage: true,
    });
  });

  test('Glassmorphism cards render correctly', async ({ page }) => {
    await page.goto('/');

    const statsCard = page.locator('.ocean-card').first();

    // Verify glassmorphism (or fallback) is applied
    const background = await statsCard.evaluate(el =>
      window.getComputedStyle(el).background
    );
    expect(background).toContain('rgba');

    // Hover state screenshot
    await statsCard.hover();
    await page.waitForTimeout(500); // Let shimmer animation start
    await expect(statsCard).toHaveScreenshot('card-hover.png');
  });
});
```

**GitHub Actions Workflow**:

```yaml
# .github/workflows/playwright.yml (auto-generated by Playwright CLI)
name: Playwright Tests
on:
  push:
    branches: [ main, 003-ocean-sapphire-theme ]
  pull_request:
    branches: [ main ]
jobs:
  test:
    timeout-minutes: 60
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v4
    - uses: actions/setup-node@v4
      with:
        node-version: 18
    - name: Install dependencies
      run: npm ci
    - name: Install Playwright Browsers
      run: npx playwright install chromium --with-deps
    - name: Build Docusaurus site
      run: npm run build
    - name: Serve and test
      run: |
        npx serve build -l 3000 &
        npx playwright test
    - uses: actions/upload-artifact@v4
      if: always()
      with:
        name: playwright-report
        path: playwright-report/
        retention-days: 30
```

**Handling Screenshot Baseline Updates**:

When Ocean Sapphire CSS changes intentionally (e.g., adjusting cyan glow intensity):

```bash
# Locally: Update baselines after verifying changes are correct
npx playwright test --update-snapshots

# Commit new baseline screenshots
git add tests/visual-regression/*.png
git commit -m "Update Ocean Sapphire visual regression baselines"
```

**Important Considerations**:
- **OS Consistency**: Playwright baselines are OS-specific. If team develops on macOS but CI runs Ubuntu, screenshots will differ. Solution: Generate baselines in Docker or GitHub Actions environment.
- **Browser Choice**: Use Chromium only (not Firefox/WebKit) to avoid cross-browser rendering differences. Ocean Sapphire targets Chrome/Edge as primary browsers.
- **Threshold Tuning**: Set `maxDiffPixels: 100` to tolerate anti-aliasing differences in text rendering and CSS animations.

**Best Practices**:
- **Baseline Storage**: Commit screenshot baselines to Git for version control and PR review
- **Test Scope**: Test 4 pages minimum - landing page + 3 diverse chapter pages (code blocks, TOC, highlight boxes)
- **Automated CSS Validation**: Add separate test to verify CSS variables are defined in compiled stylesheet
- **Contrast Ratio Checks**: Use Playwright to extract computed colors and validate WCAG AAA ratios programmatically

**Sources**:
- [Visual Regression Testing using Playwright and GitHub Actions](https://www.duncanmackenzie.net/blog/visual-regression-testing/)
- [Automating visual UI tests with Playwright and GitHub Actions](https://mmazzarolo.com/blog/2022-09-09-visual-regression-testing-with-playwright-and-github-actions/)
- [Setting up CI | Playwright](https://playwright.dev/docs/ci-intro)
- [Percy vs Chromatic comparison](https://www.chromatic.com/compare/percy)

---

## Decision 4: CSS Variable Fallback Pattern (Inline Dual Declaration)

**Decision**: Use inline fallback pattern with hard-coded value followed by CSS variable: `color: #b8d4ff; color: var(--ocean-text-pale);`

**Rationale**:
- **Graceful Degradation**: Browsers without CSS variable support (IE 11, very old Chrome/Firefox) use the first declaration; modern browsers override with the CSS variable.
- **Standards Compliance**: This is the W3C-recommended pattern for CSS variable fallbacks, documented in MDN's "Using CSS custom properties" guide.
- **No JavaScript Required**: Pure CSS solution avoids runtime detection overhead and works even if JavaScript is disabled.
- **Maintainability**: When Ocean Sapphire colors change, updating CSS variables in `:root` automatically propagates to all modern browsers; only IE 11 users see stale fallback colors (acceptable tradeoff given IE 11 EOL in 2022).

**Alternatives Considered**:
1. **`var()` function with fallback parameter: `color: var(--ocean-text-pale, #b8d4ff)`** - Rejected because this only provides fallback when the variable is *undefined*, not when the browser doesn't support CSS variables at all. Old browsers would ignore the entire `var()` function.
2. **`@supports (--css-vars: '')` feature detection** - Rejected because less reliable than inline fallback pattern and more verbose (requires wrapping all variable usage in @supports blocks).
3. **JavaScript polyfill for CSS variables** - Rejected because adds runtime overhead and doesn't work if JavaScript is disabled. CSS-only solution is more robust.

**Implementation Pattern**:

```css
/* website/src/css/custom.css */

/* Define Ocean Sapphire CSS variables in :root */
:root {
  /* Primary Blues */
  --ocean-primary-deep: #001529;
  --ocean-mid-blue: #002140;
  --ocean-light-blue: #003a6d;

  /* Accent Cyans */
  --ocean-accent-cyan: #0096ff;
  --ocean-soft-cyan: #00d4ff;

  /* Text Colors */
  --ocean-text-white: #ffffff;
  --ocean-text-pale: #b8d4ff;

  /* UI Elements */
  --ocean-card-bg: rgba(0, 50, 100, 0.3);
  --ocean-border-glow: rgba(0, 150, 255, 0.3);
}

/* Apply with inline fallback pattern */
.ocean-heading {
  /* Fallback for browsers without CSS variables */
  color: #ffffff;
  /* Modern browsers use CSS variable */
  color: var(--ocean-text-white);

  text-shadow: 0 0 30px rgba(0, 150, 255, 0.6);
  text-shadow: 0 0 30px var(--ocean-border-glow);
}

.ocean-body-text {
  color: #b8d4ff; /* IE 11 fallback */
  color: var(--ocean-text-pale); /* Modern browsers */

  font-family: Georgia, serif;
  font-size: 1.1em;
  line-height: 2.0;
}

.ocean-button {
  background: rgba(0, 150, 255, 0.2); /* Fallback */
  background: var(--ocean-card-bg); /* CSS variable */

  border: 1px solid #0096ff; /* Fallback */
  border: 1px solid var(--ocean-accent-cyan); /* CSS variable */
}
```

**Variable Naming Convention**:

| Category | Pattern | Example |
|----------|---------|---------|
| Colors | `--ocean-[category]-[variant]` | `--ocean-primary-deep` |
| Typography | `--ocean-font-[purpose]` | `--ocean-font-heading` |
| Spacing | `--ocean-space-[size]` | `--ocean-space-lg` |
| Effects | `--ocean-effect-[name]` | `--ocean-glow-cyan` |

**Best Practices**:
- **Always Duplicate**: Every CSS variable usage must have a hard-coded fallback on the line immediately before
- **Fallback Accuracy**: Fallback value should exactly match the CSS variable's value to ensure visual consistency
- **Color Format**: Use consistent color format for fallbacks (hex codes for solid colors, `rgba()` for transparency)
- **Grouping**: Define all Ocean Sapphire variables in single `:root` block at top of `custom.css` for easy maintenance
- **Documentation**: Add CSS comments explaining variable purpose: `/* --ocean-text-pale: Body text in dark mode */`

**Why Not `var(--name, fallback)` Syntax?**:

The `var()` function's second parameter only works when:
- The variable name is valid but undefined
- The browser supports CSS variables

It does NOT work when:
- The browser doesn't recognize `var()` function at all (IE 11, old Chrome)
- In these cases, the entire property is ignored

Example of **incorrect** pattern:
```css
/* ❌ WRONG: IE 11 ignores entire line, text becomes black (browser default) */
.ocean-text {
  color: var(--ocean-text-pale, #b8d4ff);
}
```

Example of **correct** pattern:
```css
/* ✅ CORRECT: IE 11 uses first line, modern browsers override with second line */
.ocean-text {
  color: #b8d4ff;
  color: var(--ocean-text-pale);
}
```

**Sources**:
- [Using CSS custom properties (variables) - CSS | MDN](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_cascading_variables/Using_CSS_custom_properties)
- [var() - CSS | MDN](https://developer.mozilla.org/en-US/docs/Web/CSS/Reference/Values/var)
- [Defensive CSS - CSS Variable Fallback](https://defensivecss.dev/tip/css-variable-fallback/)
- [CSS Variable Usage: Complete Guide | CodeLucky](https://codelucky.com/css-variable-usage/)

---

## Decision 5: Docusaurus Component Swizzling (TOC & Code Block Customization)

**Decision**: Swizzle `TOC` (Table of Contents) and `CodeBlock/Content` components using "wrap" mode to customize Ocean Sapphire active states and copy button styling

**Rationale**:
- **Safe Customization**: "Wrap" mode allows enhancing components without copying entire theme code, making Docusaurus upgrades safer (less code to maintain).
- **Active State Control**: TOC component needs custom CSS for cyan glow and left border on active chapter (FR-009). Swizzling provides access to active state styling hooks.
- **Copy Button Styling**: Code block copy button requires cyan hover effect (FR-012). Swizzling allows overriding default button styles.
- **Official Pattern**: Swizzling is Docusaurus's recommended method for theme customization without forking the entire theme.

**Alternatives Considered**:
1. **Global CSS overrides without swizzling** - Rejected because Docusaurus uses CSS Modules with hashed class names (`.tocItem__e4KJ`), making global overrides fragile and version-dependent.
2. **Eject mode (full component copy)** - Rejected because copies large amounts of internal code that must be maintained during Docusaurus upgrades. "Wrap" mode is more maintainable.
3. **Custom React components from scratch** - Rejected because duplicates Docusaurus functionality and loses integration with Docusaurus routing/navigation.

**Components to Swizzle**:

### 1. TOC Component (Ocean Sapphire Active State)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic TOC -- --wrap
```

**Creates**: `website/src/theme/TOC/index.tsx`

**Implementation**:
```tsx
// website/src/theme/TOC/index.tsx
import React from 'react';
import TOC from '@theme-original/TOC';
import type TOCType from '@theme/TOC';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof TOCType>;

export default function TOCWrapper(props: Props): JSX.Element {
  return (
    <div className="ocean-toc-wrapper">
      <TOC {...props} />
    </div>
  );
}
```

**Styling** (`website/src/css/custom.css`):
```css
/* Ocean Sapphire TOC Active State */
.ocean-toc-wrapper .table-of-contents__link--active {
  border-left: 3px solid #0096ff;
  border-left: 3px solid var(--ocean-accent-cyan);

  padding-left: 1rem;

  box-shadow: 0 0 15px rgba(0, 150, 255, 0.3);
  box-shadow: 0 0 15px var(--ocean-border-glow);

  color: #00d4ff;
  color: var(--ocean-soft-cyan);

  font-weight: 500;
}

/* Smooth transition for active state changes */
.ocean-toc-wrapper .table-of-contents__link {
  transition: all 0.3s ease;
}
```

### 2. CodeBlock Component (Copy Button Ocean Sapphire Styling)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic CodeBlock/Content -- --wrap
```

**Creates**: `website/src/theme/CodeBlock/Content/index.tsx`

**Implementation**:
```tsx
// website/src/theme/CodeBlock/Content/index.tsx
import React from 'react';
import Content from '@theme-original/CodeBlock/Content';
import type ContentType from '@theme/CodeBlock/Content';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return (
    <div className="ocean-code-block">
      <Content {...props} />
    </div>
  );
}
```

**Styling** (`website/src/css/custom.css`):
```css
/* Ocean Sapphire Code Block Styling */
.ocean-code-block {
  /* Fira Code font stack with automatic fallback */
  font-family: 'Fira Code', 'Courier New', Consolas, monospace;
  font-size: 0.95em;
}

/* Copy button Ocean Sapphire hover effect */
.ocean-code-block button[class*="copyButton"] {
  background: rgba(0, 50, 100, 0.3);
  background: var(--ocean-card-bg);

  border: 1px solid #0096ff;
  border: 1px solid var(--ocean-accent-cyan);

  color: #b8d4ff;
  color: var(--ocean-text-pale);

  transition: all 0.3s ease;
}

.ocean-code-block button[class*="copyButton"]:hover {
  box-shadow: 0 0 20px rgba(0, 150, 255, 0.5);
  box-shadow: 0 0 20px var(--ocean-border-glow);

  background: rgba(0, 150, 255, 0.2);

  color: #00d4ff;
  color: var(--ocean-soft-cyan);
}

/* Copied state feedback */
.ocean-code-block button[class*="copyButton"][class*="copyButtonCopied"] {
  background: rgba(0, 150, 255, 0.3);
  border-color: #00d4ff;
  border-color: var(--ocean-soft-cyan);
}
```

**Swizzle Safety Levels**:

| Component | Safety | Notes |
|-----------|--------|-------|
| TOC | ✅ Safe | Wrapper adds minimal customization, safe to upgrade |
| CodeBlock/Content | ⚠️ Unsafe | Contains internal logic, monitor during Docusaurus upgrades |
| DocSidebar | ⚠️ Unsafe | Complex component, prefer CSS overrides if possible |
| Navbar | ✅ Safe | Wrapper is safe, eject is unsafe |

**Best Practices**:
- **Prefer Wrap Over Eject**: "Wrap" mode is safer for upgrades; only eject if absolutely necessary
- **Minimal Logic**: Keep swizzled components as thin wrappers; push styling to CSS files
- **Version Pinning**: Document Docusaurus version when swizzling: `"@docusaurus/core": "3.1.0"`
- **Upgrade Testing**: After Docusaurus upgrades, verify swizzled components still work correctly
- **Alternative CSS Selectors**: If Docusaurus exposes stable class names or data attributes, use those instead of swizzling

**Troubleshooting**:

If swizzling breaks after Docusaurus upgrade:
1. Check Docusaurus changelog for breaking changes to swizzled components
2. Re-run swizzle command to see if component structure changed
3. Compare new component signature with existing wrapper
4. Consider moving customization to global CSS if component API is unstable

**Sources**:
- [Swizzling | Docusaurus](https://docusaurus.io/docs/swizzling)
- [Modifying copy button behaviour in code blocks | Docusaurus Discussion](https://github.com/facebook/docusaurus/discussions/5227)
- [Customization use-cases | Docusaurus Discussion](https://github.com/facebook/docusaurus/discussions/5468)

---

## Summary of Key Decisions

| # | Question | Decision | Technology/Approach | Rationale |
|---|----------|----------|---------------------|-----------|
| 1 | Three.js + SSR | Use `<BrowserOnly>` wrapper | Docusaurus Client API | Prevents build failures, official SSR-safe pattern |
| 2 | Glassmorphism fallback | `@supports` with solid backgrounds | CSS feature queries | Progressive enhancement, no JS required |
| 3 | Visual regression testing | Playwright + GitHub Actions | Open-source, free tier | Cost-effective, adequate for Ocean Sapphire needs |
| 4 | CSS variable fallbacks | Inline dual declaration | `color: #hex; color: var(--name);` | Graceful degradation for IE 11, W3C-recommended |
| 5 | Theme customization | Swizzle TOC + CodeBlock | Wrap mode (safe upgrades) | Official pattern, minimal maintenance overhead |

---

## Implementation Notes

### File Structure for Ocean Sapphire Implementation

```
website/
├── src/
│   ├── css/
│   │   ├── custom.css               # Ocean Sapphire CSS variables + component styles
│   │   └── animations.css           # Shimmer, fade-in, glow animations
│   ├── components/
│   │   ├── ThreeScene.tsx           # Three.js landing page animation
│   │   ├── OceanCard.tsx            # Glassmorphism card component
│   │   └── OceanButton.tsx          # CTA button with cyan glow
│   ├── pages/
│   │   └── index.tsx                # Custom landing page (Ocean Sapphire from scratch)
│   └── theme/                       # Swizzled components
│       ├── TOC/
│       │   └── index.tsx            # TOC wrapper for active state styling
│       └── CodeBlock/
│           └── Content/
│               └── index.tsx        # Code block wrapper for copy button
├── docusaurus.config.ts             # Docusaurus configuration
└── package.json                     # Dependencies (three, @types/three)

tests/
└── visual-regression/
    ├── ocean-sapphire.spec.ts       # Playwright visual tests
    └── screenshots/                 # Baseline screenshots (committed to Git)
        ├── landing-page.png
        ├── chapter-page.png
        └── card-hover.png

.github/
└── workflows/
    └── playwright.yml               # CI/CD visual regression pipeline
```

### CSS Architecture

**Layer 1: CSS Variables** (`:root` in `custom.css`)
- All Ocean Sapphire colors, spacing, typography as CSS variables
- Inline fallback values for each variable usage

**Layer 2: Base Styles** (`custom.css`)
- Typography (Georgia headings/body, Fira Code monospace)
- Background gradients
- Global animations

**Layer 3: Component Styles** (`custom.css` + swizzled components)
- Glassmorphism cards
- TOC active state
- Code block copy button
- Highlight boxes

**Layer 4: Animation Definitions** (`animations.css`)
- `@keyframes shimmer`
- `@keyframes fadeIn`
- `@keyframes glowPulse`
- `@media (prefers-reduced-motion: reduce)` overrides

### Dependencies to Add

```json
{
  "dependencies": {
    "three": "^0.160.0"
  },
  "devDependencies": {
    "@types/three": "^0.160.0",
    "@playwright/test": "^1.41.0"
  }
}
```

### CSS Variables Reference (Complete Set)

```css
:root {
  /* Primary Blues (backgrounds, structure) */
  --ocean-primary-deep: #001529;
  --ocean-mid-blue: #002140;
  --ocean-light-blue: #003a6d;

  /* Accent Cyans (interactive elements, highlights) */
  --ocean-accent-cyan: #0096ff;
  --ocean-soft-cyan: #00d4ff;

  /* Text Colors */
  --ocean-text-white: #ffffff;
  --ocean-text-pale: #b8d4ff;
  --ocean-text-caption: #8bb3e0;

  /* UI Element Colors */
  --ocean-card-bg: rgba(0, 50, 100, 0.3);
  --ocean-card-bg-fallback: rgba(0, 50, 100, 0.7);
  --ocean-border-glow: rgba(0, 150, 255, 0.3);

  /* Typography */
  --ocean-font-heading: Georgia, serif;
  --ocean-font-body: Georgia, serif;
  --ocean-font-code: 'Fira Code', 'Courier New', Consolas, monospace;

  /* Spacing (consistent with Docusaurus) */
  --ocean-space-xs: 0.5rem;
  --ocean-space-sm: 1rem;
  --ocean-space-md: 1.5rem;
  --ocean-space-lg: 2rem;
  --ocean-space-xl: 3rem;

  /* Effects */
  --ocean-glow-cyan: 0 0 20px rgba(0, 150, 255, 0.5);
  --ocean-glow-strong: 0 0 30px rgba(0, 150, 255, 0.6);
  --ocean-blur-glass: blur(10px);

  /* Animation Durations */
  --ocean-transition-fast: 0.3s;
  --ocean-transition-medium: 0.6s;
  --ocean-shimmer-duration: 3s;
}
```

### Testing Checklist

Before deploying Ocean Sapphire:

- [ ] **Three.js**: Landing page renders 3D scene at 60fps in Chrome/Firefox/Safari
- [ ] **SSR Safety**: `npm run build` completes without "window is not defined" errors
- [ ] **Glassmorphism**: Verify blur effect in Chrome 120+, solid background fallback in Firefox 88
- [ ] **CSS Variables**: All Ocean Sapphire colors defined in `:root`, no hard-coded hex values in components
- [ ] **Visual Regression**: Playwright tests pass for landing page + 3 chapter pages
- [ ] **Accessibility**: WCAG AAA contrast ratios verified programmatically (≥7:1 for body text)
- [ ] **Responsive**: Test on mobile viewports (390px iPhone 12, 360px Galaxy S21)
- [ ] **prefers-reduced-motion**: Animations disabled when user preference detected
- [ ] **Swizzled Components**: TOC active state shows cyan glow, code copy button has hover effect
- [ ] **Font Loading**: Fira Code falls back to Courier New without FOUT (Flash of Unstyled Text)

---

## All Research Decisions Align With

- ✅ **FR-001 to FR-019**: All functional requirements have implementation patterns documented
- ✅ **SC-011**: Visual regression testing strategy defined (Playwright)
- ✅ **Constitution v1.1.0**: Ocean Sapphire design specifications matched
- ✅ **Accessibility**: WCAG AAA compliance through contrast ratios and `prefers-reduced-motion`
- ✅ **Performance**: <3 second landing page load, 60fps animations
- ✅ **Browser Compatibility**: Graceful degradation for IE 11, Firefox <103, Safari <15.4

**No unresolved NEEDS CLARIFICATION items** - All technical decisions finalized.

**Ready for Implementation**: `plan.md` (architecture decisions), `tasks.md` (user story breakdown)
