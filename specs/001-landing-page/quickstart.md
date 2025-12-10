# Quickstart: Professional Landing Page

**Feature**: 001-landing-page
**Audience**: Developers implementing this feature
**Prerequisites**: Node.js 18+, Docusaurus 3.x project initialized

## Overview

This guide walks through setting up, developing, and testing the Professional Landing Page feature. Follow steps sequentially for fastest time-to-first-render.

## Setup (5 minutes)

### 1. Install Dependencies

```bash
# Core dependencies
npm install three @react-three/fiber @react-three/drei framer-motion clsx

# Dev dependencies (testing)
npm install -D @types/three @testing-library/react @testing-library/jest-dom playwright @playwright/test
```

### 2. Configure Tailwind CSS (if not already configured)

```bash
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

Update `tailwind.config.js`:
```js
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      colors: {
        'cyan-neon': '#00F0FF',
        'magenta-neon': '#FF2A6D',
        'dark-bg': '#0A0A0F',
        'light-bg': '#F8F9FB',
        'card-dark': '#12131A',
        'text-dark': '#D6D6D6',
        'text-light': '#2A2A2A'
      },
      fontFamily: {
        inter: ['Inter', 'sans-serif'],
        poppins: ['Poppins', 'sans-serif']
      }
    }
  }
};
```

Add to `src/css/custom.css`:
```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

### 3. Add Google Fonts

Update `docusaurus.config.js`:
```js
module.exports = {
  // ...
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com'
      }
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous'
      }
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;700&family=Poppins:wght@400;600&display=swap'
      }
    }
  ]
};
```

## Development (30 minutes)

### Phase 1: Custom Hooks (5 minutes)

Create utility hooks in `src/hooks/`:

**`useMediaQuery.ts`**:
```typescript
import { useState, useEffect } from 'react';

export function useMediaQuery(query: string): boolean {
  const [matches, setMatches] = useState(false);

  useEffect(() => {
    const media = window.matchMedia(query);
    setMatches(media.matches);

    const listener = () => setMatches(media.matches);
    media.addEventListener('change', listener);
    return () => media.removeEventListener('change', listener);
  }, [query]);

  return matches;
}

// Usage: const isDesktop = useMediaQuery('(min-width: 768px)');
```

**`useReducedMotion.ts`**:
```typescript
import { useMediaQuery } from './useMediaQuery';

export function useReducedMotion(): boolean {
  return useMediaQuery('(prefers-reduced-motion: reduce)');
}
```

**`useWebGLSupport.ts`**:
```typescript
import { useState, useEffect } from 'react';

export function useWebGLSupport(): boolean {
  const [isSupported, setIsSupported] = useState(true);

  useEffect(() => {
    try {
      const canvas = document.createElement('canvas');
      const gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
      setIsSupported(!!gl);
    } catch {
      setIsSupported(false);
    }
  }, []);

  return isSupported;
}
```

### Phase 2: Fallback SVG (5 minutes)

Create `src/components/common/FallbackSVG.tsx`:
```tsx
import React from 'react';

export const FallbackSVG: React.FC = () => (
  <svg
    width="400"
    height="400"
    viewBox="0 0 400 400"
    xmlns="http://www.w3.org/2000/svg"
    role="img"
    aria-label="Animated 3D cube visualization"
  >
    <defs>
      <linearGradient id="neonGlow" x1="0%" y1="0%" x2="100%" y2="100%">
        <stop offset="0%" stopColor="#00F0FF" stopOpacity="0.8" />
        <stop offset="100%" stopColor="#FF2A6D" stopOpacity="0.6" />
      </linearGradient>
    </defs>

    {/* Cube outline with neon edges */}
    <g transform="translate(200, 200) rotateX(20) rotateY(30)">
      <path
        d="M-80,-80 L80,-80 L80,80 L-80,80 Z"
        fill="none"
        stroke="url(#neonGlow)"
        strokeWidth="3"
        opacity="0.8"
      />
      <path
        d="M-60,-100 L100,-100 L100,60 L-60,60 Z"
        fill="none"
        stroke="#00F0FF"
        strokeWidth="2"
        opacity="0.6"
      />
      <path
        d="M-80,-80 L-60,-100 M80,-80 L100,-100 M80,80 L100,60 M-80,80 L-60,60"
        stroke="#00F0FF"
        strokeWidth="2"
        opacity="0.7"
      />
    </g>
  </svg>
);
```

### Phase 3: Hero Component (10 minutes)

Create `src/components/LandingPage/Hero.tsx` and `Hero.module.css`:

**Hero.tsx**:
```tsx
import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Hero.module.css';

export const Hero: React.FC = () => {
  return (
    <div className={styles.hero}>
      <h1 className={styles.heading}>
        Physical AI & Humanoid Robotics
      </h1>
      <p className={styles.subheading}>
        From Digital Intelligence to Embodied Systems
      </p>
      <Link
        to="/docs/intro"
        className={styles.ctaButton}
        aria-label="Navigate to textbook introduction"
      >
        Read Book
      </Link>
    </div>
  );
};
```

**Hero.module.css**:
```css
.hero {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: flex-start;
  padding: 2rem;
  height: 100%;
}

.heading {
  font-family: 'Inter', sans-serif;
  font-weight: 700;
  font-size: 48px;
  line-height: 1.2;
  background: linear-gradient(135deg, #00F0FF 0%, #FF2A6D 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  margin-bottom: 1rem;
}

.subheading {
  font-family: 'Inter', sans-serif;
  font-weight: 400;
  font-size: 20px;
  color: #D6D6D6;
  margin-bottom: 2rem;
}

.ctaButton {
  width: 200px;
  height: 60px;
  background-color: #00F0FF;
  color: #000000;
  font-family: 'Inter', sans-serif;
  font-weight: 600;
  font-size: 18px;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  transition: box-shadow 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  text-decoration: none;
  animation: pulse 3s ease-in-out infinite;
}

.ctaButton:hover {
  box-shadow: 0 0 20px #00F0FF, 0 0 40px #00F0FF;
}

.ctaButton:focus-visible {
  outline: 2px solid #00F0FF;
  outline-offset: 4px;
}

@keyframes pulse {
  0%, 100% {
    box-shadow: 0 0 10px rgba(0, 240, 255, 0.3);
  }
  50% {
    box-shadow: 0 0 20px rgba(0, 240, 255, 0.6);
  }
}

@media (prefers-color-scheme: light) {
  .subheading {
    color: #6E6E6E;
  }
  .ctaButton {
    color: #003A40;
  }
}
```

### Phase 4: Stats Cards (10 minutes)

Create `src/components/LandingPage/StatCard.tsx` and `StatsSection.tsx`:

**StatCard.tsx**:
```tsx
import React from 'react';
import { motion } from 'framer-motion';
import styles from './StatCard.module.css';

interface StatCardProps {
  title: string;
  value: string;
}

export const StatCard: React.FC<StatCardProps> = ({ title, value }) => {
  return (
    <motion.div
      className={styles.card}
      whileHover={{ y: -10, boxShadow: '0 0 20px rgba(0, 240, 255, 0.6)' }}
      transition={{ duration: 0.3 }}
    >
      <h3 className={styles.value}>{value}</h3>
      <p className={styles.title}>{title}</p>
    </motion.div>
  );
};
```

**StatCard.module.css**:
```css
.card {
  background: rgba(18, 19, 26, 0.6);
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px);
  border: 1px solid rgba(0, 240, 255, 0.2);
  border-radius: 12px;
  padding: 2rem;
  text-align: center;
  transition: transform 0.3s ease, box-shadow 0.3s ease;
}

@supports not (backdrop-filter: blur(10px)) {
  .card {
    background: rgba(18, 19, 26, 0.9);
  }
}

.value {
  font-family: 'Poppins', sans-serif;
  font-weight: 600;
  font-size: 36px;
  color: #00F0FF;
  margin-bottom: 0.5rem;
}

.title {
  font-family: 'Inter', sans-serif;
  font-size: 16px;
  color: #D6D6D6;
  margin: 0;
}

@media (prefers-color-scheme: light) {
  .card {
    background: rgba(255, 255, 255, 0.8);
    border-color: rgba(0, 240, 255, 0.3);
  }
  .title {
    color: #2A2A2A;
  }
}
```

**StatsSection.tsx**:
```tsx
import React from 'react';
import { motion, useAnimation } from 'framer-motion';
import { useInView } from 'framer-motion';
import { StatCard } from './StatCard';
import styles from './StatsSection.module.css';

const stats = [
  { title: 'Modules', value: '4' },
  { title: 'Chapters', value: '21' },
  { title: 'Code Examples', value: '50+' },
  { title: 'AI-Powered Chatbot', value: '✓' }
];

export const StatsSection: React.FC = () => {
  const ref = React.useRef(null);
  const isInView = useInView(ref, { once: true, amount: 0.3 });
  const controls = useAnimation();

  React.useEffect(() => {
    if (isInView) {
      controls.start('visible');
    }
  }, [isInView, controls]);

  return (
    <motion.section
      ref={ref}
      className={styles.container}
      initial="hidden"
      animate={controls}
      variants={{
        hidden: { opacity: 0 },
        visible: {
          opacity: 1,
          transition: { staggerChildren: 0.1 }
        }
      }}
    >
      {stats.map((stat, index) => (
        <motion.div
          key={index}
          variants={{
            hidden: { opacity: 0, y: 20 },
            visible: { opacity: 1, y: 0 }
          }}
        >
          <StatCard title={stat.title} value={stat.value} />
        </motion.div>
      ))}
    </motion.section>
  );
};
```

**StatsSection.module.css**:
```css
.container {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 2rem;
  padding: 4rem 2rem;
  max-width: 1200px;
  margin: 0 auto;
}

@media (max-width: 768px) {
  .container {
    grid-template-columns: 1fr;
  }
}
```

### Phase 5: Animated Cube (Optional, see research.md for full implementation)

Due to complexity, refer to `research.md` Section 2 for complete Three.js implementation. Stub for now:

**AnimatedCube.tsx** (placeholder):
```tsx
import React from 'react';
import { Canvas } from '@react-three/fiber';
// Full implementation in tasks phase
export const AnimatedCube: React.FC = () => <div>3D Cube (TODO)</div>;
```

## Testing (10 minutes)

### Unit Tests

```bash
npm test -- Hero.test.tsx
```

### E2E Tests (Playwright)

```bash
npx playwright test landing-page.spec.ts
```

### Lighthouse CI

```bash
npm run build
npx lhci autorun
```

## Validation Checklist

- [ ] Page loads at `/` (root path)
- [ ] Heading displays cyan-to-magenta gradient
- [ ] Button navigates to `/docs/intro` without page reload
- [ ] Button glows on hover
- [ ] Stats cards appear with stagger animation on scroll
- [ ] Mobile layout stacks vertically
- [ ] Keyboard navigation works (Tab to button, Enter to navigate)
- [ ] Lighthouse Performance >90
- [ ] Lighthouse Accessibility >90
- [ ] Reduced motion preference disables animations

## Troubleshooting

**Issue**: Three.js bundle too large
- **Solution**: Ensure lazy loading with `React.lazy()` and desktop-only rendering

**Issue**: Glass-morphism not working in Firefox
- **Solution**: Check `@supports` fallback in CSS

**Issue**: Button doesn't navigate
- **Solution**: Use `Link` from `@docusaurus/Link`, not `<a>` tag

**Issue**: Animations janky
- **Solution**: Use GPU-accelerated properties only (`transform`, `opacity`), avoid `width`/`height`

## Next Steps

Run `/sp.tasks` to generate actionable task list for implementation.
