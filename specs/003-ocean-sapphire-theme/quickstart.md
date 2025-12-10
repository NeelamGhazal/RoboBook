# Quickstart Guide: Ocean Sapphire Design System Implementation

**Feature**: 003-ocean-sapphire-theme | **Date**: 2025-12-10
**Purpose**: Developer setup and implementation guide for Ocean Sapphire theme on Docusaurus site

---

## Prerequisites

Before implementing Ocean Sapphire, ensure you have:

- **Node.js**: 18.x or later
- **npm**: 9.x or later
- **Docusaurus**: 3.x installed and running
- **Git**: For version control and committing baselines
- **Modern Browser**: Chrome 90+, Firefox 88+, or Safari 14+ for testing

---

## Quick Start (5 Minutes)

### 1. Install Dependencies

```bash
cd website/

# Install Three.js for landing page animation
npm install three @types/three

# Install Playwright for visual regression testing (optional, for CI/CD)
npm install --save-dev @playwright/test
```

### 2. Create Ocean Sapphire CSS Variables

Create or modify `website/src/css/custom.css`:

```css
/* Ocean Sapphire Design System Variables */
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

/* Apply Ocean Sapphire to Docusaurus */
[data-theme='dark'] {
  --ifm-background-color: #001529;
  --ifm-background-color: var(--ocean-primary-deep);

  --ifm-font-color-base: #b8d4ff;
  --ifm-font-color-base: var(--ocean-text-pale);
}
```

### 3. Create Landing Page Component

Create `website/src/pages/index.tsx`:

```tsx
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Layout from '@theme/Layout';

export default function LandingPage() {
  return (
    <Layout>
      <div className="ocean-landing" style={{
        background: 'linear-gradient(180deg, #001529 0%, #002140 50%, #003a6d 100%)',
        minHeight: '100vh',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
      }}>
        <div>
          <h1 style={{
            fontFamily: 'Georgia, serif',
            fontSize: '4.5em',
            color: '#ffffff',
            textShadow: '0 0 30px rgba(0, 150, 255, 0.6)',
          }}>
            HUMARIDE
          </h1>
          <p style={{ color: '#b8d4ff' }}>Physical AI & Humanoid Robotics</p>
        </div>

        <BrowserOnly fallback={<div>Loading 3D scene...</div>}>
          {() => {
            const ThreeScene = require('../components/ThreeScene').default;
            return <ThreeScene width={800} height={400} />;
          }}
        </BrowserOnly>
      </div>
    </Layout>
  );
}
```

### 4. Test Ocean Sapphire

```bash
# Start Docusaurus dev server
npm run start

# Open browser to http://localhost:3000
# Verify Ocean Sapphire colors, gradient background, and Three.js scene
```

---

## Full Implementation Guide

### Step 1: Project Setup

#### 1.1 Install Three.js

```bash
cd website/
npm install three@^0.160.0 @types/three@^0.160.0
```

**Why**: Three.js powers the landing page animated cube with Ocean Sapphire gradient edges.

#### 1.2 Verify Docusaurus Configuration

Check `website/docusaurus.config.ts`:

```typescript
import {themes as prismThemes} from 'prism-react-renderer';

const config: Config = {
  // ... other config
  themeConfig: {
    colorMode: {
      defaultMode: 'dark',  // Ocean Sapphire is dark theme only
      disableSwitch: true,  // No light/dark toggle
      respectPrefersColorScheme: false,
    },
  },
};
```

---

### Step 2: CSS Architecture

#### 2.1 Create Ocean Sapphire Variables

**File**: `website/src/css/custom.css`

Copy the complete CSS variable set from `specs/003-ocean-sapphire-theme/contracts/OceanSapphireVariables.css`.

**Important**: Use inline fallback pattern for every CSS variable:

```css
.example {
  color: #b8d4ff; /* Fallback for IE 11 */
  color: var(--ocean-text-pale); /* Modern browsers */
}
```

#### 2.2 Create Animation Keyframes

**File**: `website/src/css/animations.css`

```css
/* Shimmer Animation (Card Hover) */
@keyframes shimmer {
  0% { background-position: -200% center; }
  100% { background-position: 200% center; }
}

/* Fade-In Animation (Page Transitions) */
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Glow Pulse Animation (TOC Active State) */
@keyframes glowPulse {
  0%, 100% { box-shadow: 0 0 15px rgba(0, 150, 255, 0.3); }
  50% { box-shadow: 0 0 25px rgba(0, 150, 255, 0.5); }
}

/* Accessibility: Disable animations for users with motion sensitivity */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

#### 2.3 Create Glassmorphism Styles

**File**: `website/src/css/glassmorphism.css`

```css
/* Ocean Card Base Styles */
.ocean-card {
  background: rgba(0, 50, 100, 0.7); /* Fallback for no backdrop-filter */
  border: 1px solid rgba(0, 150, 255, 0.3);
  border-radius: 12px;
  padding: 2rem;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
  transition: all 0.3s ease;
}

/* Glassmorphism for Modern Browsers */
@supports (backdrop-filter: blur(10px)) {
  .ocean-card {
    background: rgba(0, 50, 100, 0.3); /* Less opaque with blur */
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px); /* Safari 15.4-17 */
  }
}

/* Hover State with Shimmer */
.ocean-card:hover {
  box-shadow: 0 10px 50px rgba(0, 150, 255, 0.3);
  transform: translateY(-5px);
  animation: shimmer 3s linear infinite;
  background: linear-gradient(
    90deg,
    rgba(0, 50, 100, 0.3) 0%,
    rgba(0, 150, 255, 0.2) 50%,
    rgba(0, 50, 100, 0.3) 100%
  );
  background-size: 200% 100%;
}
```

#### 2.4 Import CSS in Docusaurus

**File**: `website/docusaurus.config.ts`

```typescript
const config: Config = {
  stylesheets: [
    '/css/custom.css',
    '/css/animations.css',
    '/css/glassmorphism.css',
  ],
};
```

---

### Step 3: React Components

#### 3.1 Three.js Scene Component

**File**: `website/src/components/ThreeScene.tsx`

```tsx
import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface ThreeSceneProps {
  width?: number;
  height?: number;
  rotationSpeed?: number;
}

export default function ThreeScene({
  width,
  height = 400,
  rotationSpeed = 0.01,
}: ThreeSceneProps): JSX.Element {
  const mountRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!mountRef.current) return;

    // Scene setup
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(
      75,
      (width || mountRef.current.clientWidth) / height,
      0.1,
      1000
    );
    const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });

    renderer.setSize(width || mountRef.current.clientWidth, height);
    mountRef.current.appendChild(renderer.domElement);

    // Ocean Sapphire gradient cube
    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshBasicMaterial({
      color: 0x0096ff, // Ocean Sapphire accent cyan
      wireframe: true,
    });
    const cube = new THREE.Mesh(geometry, material);
    scene.add(cube);

    camera.position.z = 5;

    // Animation loop (60fps)
    const animate = () => {
      requestAnimationFrame(animate);
      cube.rotation.x += rotationSpeed;
      cube.rotation.y += rotationSpeed;
      renderer.render(scene, camera);
    };
    animate();

    // Cleanup
    return () => {
      renderer.dispose();
      if (mountRef.current) {
        mountRef.current.removeChild(renderer.domElement);
      }
    };
  }, [width, height, rotationSpeed]);

  return <div ref={mountRef} style={{ width: width || '100%', height }} />;
}
```

#### 3.2 Ocean Card Component

**File**: `website/src/components/OceanCard.tsx`

```tsx
import React, { ReactNode } from 'react';

interface OceanCardProps {
  children: ReactNode;
  className?: string;
  enableShimmer?: boolean;
}

export default function OceanCard({
  children,
  className = '',
  enableShimmer = true,
}: OceanCardProps): JSX.Element {
  return (
    <div className={`ocean-card ${enableShimmer ? 'ocean-card--shimmer' : ''} ${className}`}>
      {children}
    </div>
  );
}
```

#### 3.3 Stats Card Component

**File**: `website/src/components/StatsCard.tsx`

```tsx
import React from 'react';
import OceanCard from './OceanCard';

interface StatsCardProps {
  number: string;
  label: string;
  icon?: string;
}

export default function StatsCard({ number, label, icon }: StatsCardProps): JSX.Element {
  return (
    <OceanCard>
      {icon && <div className="ocean-card__icon">{icon}</div>}
      <div className="ocean-card__number" style={{ fontSize: '2.5em', color: '#00d4ff' }}>
        {number}
      </div>
      <div className="ocean-card__label" style={{ fontSize: '0.9em', color: '#8bb3e0' }}>
        {label}
      </div>
    </OceanCard>
  );
}
```

---

### Step 4: Docusaurus Theme Customization

#### 4.1 Swizzle TOC Component

```bash
cd website/
npm run swizzle @docusaurus/theme-classic TOC -- --wrap
```

**File**: `website/src/theme/TOC/index.tsx`

```tsx
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

**CSS**: `website/src/css/custom.css`

```css
/* Ocean Sapphire TOC Active State */
.ocean-toc-wrapper .table-of-contents__link--active {
  border-left: 3px solid #0096ff;
  padding-left: 1rem;
  box-shadow: 0 0 15px rgba(0, 150, 255, 0.3);
  color: #00d4ff;
  font-weight: 500;
  animation: glowPulse 2s ease-in-out infinite;
}
```

#### 4.2 Swizzle CodeBlock Component

```bash
npm run swizzle @docusaurus/theme-classic CodeBlock/Content -- --wrap
```

**File**: `website/src/theme/CodeBlock/Content/index.tsx`

```tsx
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

**CSS**: `website/src/css/custom.css`

```css
/* Ocean Sapphire Code Block Copy Button */
.ocean-code-block button[class*="copyButton"] {
  background: rgba(0, 50, 100, 0.3);
  border: 1px solid #0096ff;
  color: #b8d4ff;
  transition: all 0.3s ease;
}

.ocean-code-block button[class*="copyButton"]:hover {
  box-shadow: 0 0 20px rgba(0, 150, 255, 0.5);
  background: rgba(0, 150, 255, 0.2);
  color: #00d4ff;
}
```

---

### Step 5: Visual Regression Testing

#### 5.1 Initialize Playwright

```bash
cd website/
npm init playwright@latest
```

Choose options:
- Language: TypeScript
- Test folder: `tests`
- Add GitHub Actions workflow: Yes
- Install browsers: Chromium only

#### 5.2 Create Visual Tests

**File**: `tests/visual-regression/ocean-sapphire.spec.ts`

```typescript
import { test, expect } from '@playwright/test';

test.describe('Ocean Sapphire Visual Regression', () => {
  test('Landing page renders correctly', async ({ page }) => {
    await page.goto('/');
    await page.waitForSelector('.ocean-landing', { state: 'visible' });

    await expect(page).toHaveScreenshot('landing-page.png', {
      fullPage: true,
      maxDiffPixels: 100,
    });
  });

  test('TOC active state has cyan glow', async ({ page }) => {
    await page.goto('/docs/intro');

    const tocActive = page.locator('.table-of-contents__link--active');
    await expect(tocActive).toHaveCSS('border-left', '3px solid rgb(0, 150, 255)');
  });
});
```

#### 5.3 Generate Baseline Screenshots

```bash
# Build Docusaurus site
npm run build

# Serve locally
npx serve build -l 3000 &

# Generate baseline screenshots
npx playwright test --update-snapshots

# Commit baselines to Git
git add tests/visual-regression/*.png
git commit -m "Add Ocean Sapphire visual regression baselines"
```

---

### Step 6: Build & Deploy

#### 6.1 Local Build Test

```bash
cd website/
npm run build

# Verify no SSR errors (Three.js properly wrapped in BrowserOnly)
# Check build output for warnings
```

#### 6.2 Deploy to GitHub Pages

```bash
# Configure docusaurus.config.ts
const config: Config = {
  url: 'https://yourusername.github.io',
  baseUrl: '/yourrepo/',
  organizationName: 'yourusername',
  projectName: 'yourrepo',
};

# Deploy
npm run deploy
```

---

## Testing Checklist

Before considering Ocean Sapphire complete:

- [ ] **Dev Server**: `npm run start` runs without errors
- [ ] **Three.js**: Landing page cube animates at 60fps, no console errors
- [ ] **CSS Variables**: All Ocean Sapphire colors visible in computed styles
- [ ] **Glassmorphism**: Blur effect visible in Chrome 120+, solid fallback in Firefox 88
- [ ] **TOC Active State**: Cyan left border + glow on active chapter
- [ ] **Code Block**: Copy button has cyan hover effect
- [ ] **Visual Regression**: Playwright tests pass with baseline screenshots
- [ ] **Build**: `npm run build` completes without SSR errors
- [ ] **Mobile**: Test on iPhone 12 (390px) and Galaxy S21 (360px) viewports
- [ ] **Accessibility**: `prefers-reduced-motion` disables animations
- [ ] **Contrast**: WCAG AAA ratios verified (body text 8.2:1, headings 14.5:1)

---

## Troubleshooting

### Issue: "ReferenceError: window is not defined" during build

**Solution**: Ensure Three.js component is wrapped in `<BrowserOnly>`:

```tsx
<BrowserOnly fallback={<div>Loading...</div>}>
  {() => {
    const ThreeScene = require('../components/ThreeScene').default;
    return <ThreeScene />;
  }}
</BrowserOnly>
```

### Issue: Glassmorphism not working (no blur effect)

**Check**: Browser supports `backdrop-filter`:
- Chrome 76+, Firefox 103+, Safari 15.4+

**Solution**: Fallback solid background should appear automatically via `@supports` query.

### Issue: Playwright screenshots don't match baselines

**Solution**: Regenerate baselines in same environment (OS, browser version):

```bash
npx playwright test --update-snapshots
git add tests/**/*.png
git commit -m "Update Ocean Sapphire baselines"
```

---

## Next Steps

After Ocean Sapphire implementation:

1. **Run `/sp.tasks`**: Generate actionable task list from this plan
2. **Implement User Stories**: Follow P1→P2→P3→P4 priority order
3. **Create Pull Request**: Use `/sp.commit-pr` for automated PR creation
4. **Document Changes**: Update README with Ocean Sapphire implementation details

---

## Resources

- **Three.js Docs**: https://threejs.org/docs/
- **Docusaurus Swizzling**: https://docusaurus.io/docs/swizzling
- **Playwright Visual Testing**: https://playwright.dev/docs/test-snapshots
- **CSS backdrop-filter**: https://developer.mozilla.org/en-US/docs/Web/CSS/backdrop-filter
- **WCAG Contrast Checker**: https://webaim.org/resources/contrastchecker/

---

**Ocean Sapphire implementation time estimate**: 8-12 hours for complete P1-P4 user stories + testing
