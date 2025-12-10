# Quickstart Guide: Landing Page Redesign Implementation

**Feature**: 004-landing-page-redesign
**Date**: 2025-12-10
**Estimated Time**: 4-6 hours

## Prerequisites

Before starting implementation, ensure you have:

- [ ] Node.js 18+ installed
- [ ] Git repository cloned locally
- [ ] Branch `004-landing-page-redesign` checked out
- [ ] Docusaurus dev server running (`npm start` in `website/` directory)
- [ ] Robot image asset (520-580px, WebP, <200KB) ready
- [ ] Feature card images (3×, 3:2 ratio, WebP, <50KB each) ready
- [ ] Code editor with TypeScript support (VS Code recommended)
- [ ] Browser DevTools open (for testing responsive layouts)

---

## Implementation Checklist

### Phase 1: Setup & Assets (30 minutes)

- [ ] **1.1** Place robot image in `website/static/img/robot-halfbody.webp`
- [ ] **1.2** Place feature card images in `website/static/img/`:
  - `feature-ai-book.webp`
  - `feature-chatbot.webp`
  - `feature-personalization.webp`
- [ ] **1.3** Verify image sizes: Robot <200KB, cards <50KB each
- [ ] **1.4** Test image paths by navigating to `http://localhost:3000/img/robot-halfbody.webp`

### Phase 2: Theme System (45 minutes)

- [ ] **2.1** Open `website/src/css/custom.css`
- [ ] **2.2** Add CSS variables for dark theme at top of file:
  ```css
  :root[data-theme='dark'] {
    --bg-primary: #001529;
    --text-primary: #b8d4ff;
    --accent-cyan: #0096ff;
    --card-bg: rgba(0, 50, 100, 0.3);
    --border-color: rgba(0, 150, 255, 0.3);
  }
  ```
- [ ] **2.3** Add CSS variables for light theme:
  ```css
  :root[data-theme='light'] {
    --bg-primary: #f3f8ff;
    --text-primary: #001529;
    --accent-cyan: #0096ff;
    --card-bg: rgba(255, 255, 255, 0.6);
    --border-color: rgba(0, 150, 255, 0.3);
  }
  ```
- [ ] **2.4** Add transition rule for smooth color changes:
  ```css
  * {
    transition: background-color 200ms ease-in-out, color 200ms ease-in-out, border-color 200ms ease-in-out;
  }
  ```
- [ ] **2.5** Test: Change `document.documentElement.setAttribute('data-theme', 'light')` in browser console

### Phase 3: Hero Section (90 minutes)

- [ ] **3.1** Create `website/src/components/Hero/Hero.tsx`:
  ```tsx
  import React from 'react';
  import HeroText from './HeroText';
  import HeroImage from './HeroImage';

  const Hero: React.FC = () => {
    return (
      <div className="hero-container">
        <HeroText />
        <HeroImage />
      </div>
    );
  };

  export default Hero;
  ```
- [ ] **3.2** Create `website/src/components/Hero/HeroText.tsx` with heading (4.5rem, Orbitron), paragraph (1.3rem, Georgia), button (200×60px)
- [ ] **3.3** Create `website/src/components/Hero/HeroImage.tsx` with robot image (loading="lazy", explicit width/height)
- [ ] **3.4** Add hero styles to `custom.css`:
  - `.hero-container`: Flexbox, responsive breakpoints (50/50, 60/40, stacked)
  - Padding: 100px top, 24px side
  - Gaps: 20px heading-paragraph, 28px paragraph-button
- [ ] **3.5** Test responsive layouts:
  - Desktop (≥1024px): 50/50 side-by-side
  - Tablet (768-1023px): 60/40 layout
  - Mobile (<768px): Stacked vertically

### Phase 4: Feature Cards (90 minutes)

- [ ] **4.1** Update `website/src/components/FeatureCards/FeatureCards.tsx`:
  - Add section heading "What's Inside This Book?" (2.8rem, Georgia)
  - Add description paragraph (1.1rem, max-width 900px, centered)
  - Map over 3 feature cards
- [ ] **4.2** Update `website/src/components/FeatureCards/FeatureCard.tsx`:
  - Implement glassmorphism: `backdrop-filter: blur(10px)`, `background: var(--card-bg)`
  - 40/60 image/text split (Flexbox)
  - Card title: 1.5rem, Orbitron, 700 weight
  - Card text: 1rem, Georgia serif
  - Border: 1px solid `var(--border-color)`, 12px border-radius
  - Min height: 220px, padding: 24px
- [ ] **4.3** Add shimmer effect to `custom.css`:
  ```css
  @keyframes shimmer {
    0% { background-position: -1000px 0; }
    100% { background-position: 1000px 0; }
  }

  .feature-card::before {
    content: '';
    position: absolute;
    background: linear-gradient(90deg, transparent, var(--shimmer-color), transparent);
    animation: shimmer 3s infinite;
  }
  ```
- [ ] **4.4** Add responsive grid styles:
  - Desktop (≥1024px): 3 columns, 24px gap
  - Tablet (768-1023px): 2 columns, 16px gap
  - Mobile (<768px): 1 column, 16px gap
- [ ] **4.5** Test hover shimmer effect on desktop

### Phase 5: Navbar with Theme Toggle (60 minutes)

- [ ] **5.1** Create or update `website/src/components/Navbar/Navbar.tsx`
- [ ] **5.2** Implement fixed navbar:
  - Height: 72px
  - Logo: 40×40px icon + "RoboBook" text (Orbitron, 22px, 700 weight)
  - Menu items: "Textbook | Blog" (17px, 32px gaps)
  - Right controls: Language toggle (EN/UR), Theme toggle, GitHub icon (28px, 24px spacing)
- [ ] **5.3** Add theme toggle button:
  ```tsx
  const [theme, setTheme] = useState<'dark' | 'light'>('dark');

  const toggleTheme = () => {
    const newTheme = theme === 'dark' ? 'light' : 'dark';
    setTheme(newTheme);
    document.documentElement.setAttribute('data-theme', newTheme);
  };
  ```
- [ ] **5.4** Style navbar:
  - `position: fixed; top: 0; width: 100%; z-index: 1000;`
  - `backdrop-filter: blur(10px);`
  - Background: `var(--card-bg)`
- [ ] **5.5** Test theme toggle: Click should smoothly transition colors (200ms)

### Phase 6: Landing Page Integration (30 minutes)

- [ ] **6.1** Open `website/src/pages/index.tsx`
- [ ] **6.2** Import and render components:
  ```tsx
  import Navbar from '@site/src/components/Navbar/Navbar';
  import Hero from '@site/src/components/Hero/Hero';
  import FeatureCards from '@site/src/components/FeatureCards/FeatureCards';
  ```
- [ ] **6.3** Remove or comment out existing landing page content
- [ ] **6.4** Render new components:
  ```tsx
  export default function Home() {
    return (
      <Layout>
        <Navbar />
        <Hero />
        <FeatureCards />
      </Layout>
    );
  }
  ```
- [ ] **6.5** Load Orbitron font (Google Fonts or local):
  ```tsx
  useEffect(() => {
    const link = document.createElement('link');
    link.href = 'https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&display=swap';
    link.rel = 'stylesheet';
    document.head.appendChild(link);
  }, []);
  ```

### Phase 7: Testing & Validation (45 minutes)

- [ ] **7.1** Visual Testing:
  - [ ] All typography matches specs (4.5rem hero, 1.3rem paragraph, 1.5rem card titles)
  - [ ] Colors match Ocean Sapphire palette (dark and light themes)
  - [ ] Glassmorphism visible on cards and navbar
  - [ ] Shimmer effect animates on card hover
  - [ ] Robot image displays at correct size (520-580px)
  - [ ] Feature card images display in 3:2 ratio

- [ ] **7.2** Responsive Testing:
  - [ ] Desktop (≥1024px): Hero 50/50, cards 3 columns
  - [ ] Tablet (768-1023px): Hero 60/40, cards 2 columns
  - [ ] Mobile (<768px): Hero stacked, cards 1 column

- [ ] **7.3** Performance Testing:
  - [ ] Run Lighthouse audit: Performance >90
  - [ ] Verify image sizes: Robot <200KB, cards <50KB
  - [ ] Check Cumulative Layout Shift (CLS) = 0
  - [ ] Test page load time <3s (throttle to 3G in DevTools)

- [ ] **7.4** Accessibility Testing:
  - [ ] Use WebAIM Contrast Checker: Dark theme ≥10:1, Light theme ≥16:1
  - [ ] Tab through all interactive elements (keyboard navigation)
  - [ ] Verify alt text on all images (robot + 3 cards)
  - [ ] Test theme toggle with keyboard (Space or Enter)

- [ ] **7.5** Browser Compatibility:
  - [ ] Chrome: ✓
  - [ ] Firefox: ✓
  - [ ] Safari: ✓
  - [ ] Edge: ✓
  - [ ] Mobile Safari (iOS): ✓
  - [ ] Chrome Android: ✓

- [ ] **7.6** Constitution Compliance:
  - [ ] Orbitron ONLY on landing page (not docs/blog)
  - [ ] Files modified: ONLY index.tsx, Hero/*, FeatureCards/*, custom.css
  - [ ] No routing changes
  - [ ] No backend logic
  - [ ] No localStorage usage
  - [ ] No new dependencies added

### Phase 8: Final Review (15 minutes)

- [ ] **8.1** Review all 36 functional requirements in `spec.md` - mark each as implemented
- [ ] **8.2** Review all 14 success criteria in `spec.md` - verify each passes
- [ ] **8.3** Check console for errors (none allowed)
- [ ] **8.4** Run production build: `npm run build` in `website/` (should succeed)
- [ ] **8.5** Commit changes with descriptive message

---

## Development Commands

```bash
# Start dev server
cd website
npm start

# Run production build
npm run build

# Serve production build locally
npm run serve

# Check TypeScript types
npx tsc --noEmit
```

---

## Troubleshooting

### Issue: Glassmorphism not visible
**Solution**: Ensure `backdrop-filter: blur(10px)` is applied and elements behind cards have content. Add `-webkit-backdrop-filter: blur(10px)` for Safari.

### Issue: Theme toggle not working
**Solution**: Verify `data-theme` attribute is set on `document.documentElement`, not `document.body`. Check CSS variables are defined in `:root[data-theme='...']`.

### Issue: Images not loading
**Solution**: Verify images are in `website/static/img/` and paths start with `/img/` (not `/static/img/`). Clear browser cache.

### Issue: Orbitron font affecting docs
**Solution**: Use inline styles `style={{ fontFamily: 'Orbitron, sans-serif' }}` instead of global CSS rules. Load font only in landing page component.

### Issue: Layout shift on image load
**Solution**: Always specify explicit `width` and `height` attributes on `<img>` tags. Use `aspect-ratio` CSS property for responsive sizing.

### Issue: Responsive breakpoints not working
**Solution**: Check media queries use correct breakpoints (768px, 1024px). Test in DevTools responsive mode with exact pixel widths.

---

## Quick Reference

### Key Files
- `website/src/pages/index.tsx` - Landing page root
- `website/src/components/Hero/*` - Hero section
- `website/src/components/FeatureCards/*` - Feature cards
- `website/src/css/custom.css` - All custom styling
- `website/static/img/` - Image assets

### Exact Specifications
- **Navbar**: 72px height, 40×40px logo, 17px menu items, 32px gaps, 28px icons, 24px spacing
- **Hero Heading**: 4.5rem (72px), Orbitron 700, line-height 1.2
- **Hero Paragraph**: 1.3rem (20px), Georgia serif
- **Hero Button**: 200×60px, 1.2rem (18px) Orbitron 700
- **Section Heading**: 2.8rem (45px), Georgia serif
- **Card Title**: 1.5rem (24px), Orbitron 700
- **Card Text**: 1rem (16px), Georgia serif
- **Responsive**: 50/50 (≥1024px), 60/40 (768-1023px), stacked (<768px)
- **Theme Transition**: 200ms ease-in-out
- **Images**: Robot <200KB, cards <50KB, lazy loading

### Dark Theme Colors
- Background: `#001529`
- Text: `#b8d4ff`
- Accent: `#0096ff`
- Cards: `rgba(0, 50, 100, 0.3)`
- Borders: `rgba(0, 150, 255, 0.3)`

### Light Theme Colors
- Background: `#f3f8ff`
- Text: `#001529`
- Accent: `#0096ff`
- Cards: `rgba(255, 255, 255, 0.6)`
- Borders: `rgba(0, 150, 255, 0.3)`

---

## Next Steps After Implementation

1. Run `/sp.tasks` to generate detailed implementation tasks from this plan
2. Execute tasks in priority order (P1 → P2 → P3)
3. Test each task completion against acceptance criteria
4. Create pull request with screenshots of responsive layouts
5. Request review focusing on Constitution compliance

**Estimated Total Time**: 4-6 hours for experienced React/TypeScript developer

**Recommended Approach**: Implement in order (Phases 1-8), testing incrementally. Do not skip testing phases.
