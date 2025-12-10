# Landing Page Testing Checklist

**Feature**: 001-landing-page
**Created**: 2025-12-09
**Status**: Ready for Manual Validation

## Automated Verification ✅

### T043: Full Viewport Height Layout
- **Status**: ✅ VERIFIED
- **Implementation**: `min-height: 100vh` in index.module.css:7
- **Vertical centering**: Flexbox `align-items: center` in index.module.css:16

### T044: Mobile Responsive Breakpoint
- **Status**: ✅ VERIFIED
- **Implementation**: `@media (max-width: 768px)` in index.module.css:65
- **Behavior**: Vertical stacking, Hero on top, 3D cube hidden

### T045: Extreme Viewport Handling
- **Status**: ✅ VERIFIED
- **Small screens** (<320px): Reduced padding in index.module.css:95
- **Large screens** (>2560px): Max-width constraint (2000px) in index.module.css:102

### T046: Color Contrast Ratios
- **Status**: ✅ VERIFIED
- **Cyan (#00F0FF) on Dark (#0A0A0F)**: 14.02:1
- **WCAG 2.1 AA Minimum**: 4.5:1
- **Result**: PASS (exceeds by 3.1x)

### T047: Heading Hierarchy
- **Status**: ✅ VERIFIED
- **Main heading**: `<h1>` in Hero.tsx:19
- **Stat values**: `<h3>` in StatCard.tsx (proper nesting)

### T048: Light Mode Support
- **Status**: ✅ VERIFIED
- **Components**: Hero.module.css:78, StatCard.module.css:41, index.module.css:88
- **Implementation**: `@media (prefers-color-scheme: light)` across all components

---

## Manual Testing Required

### T049: Lighthouse CI Audit
**Target**: Performance >90, Accessibility >90

**How to Test**:
```bash
# 1. Serve the built site
npm run serve

# 2. In another terminal, run Lighthouse CI
npx lhci autorun --url=http://localhost:3000 --collect.numberOfRuns=3
```

**Expected Results**:
- Performance Score: >90
- Accessibility Score: >90
- Best Practices: >80
- SEO: >90

**Critical Metrics**:
- First Contentful Paint (FCP): <1.5s
- Time to Interactive (TTI): <3s
- Cumulative Layout Shift (CLS): <0.1

---

### T050: Keyboard Navigation
**Target**: Tab to button, Enter to navigate, visible focus indicators

**How to Test**:
1. Open http://localhost:3000 in browser
2. Press **Tab** key repeatedly
3. Verify focus moves to "Read Book" button
4. Verify cyan focus ring is visible (`:focus-visible` in Hero.module.css:57)
5. Press **Enter** on button
6. Verify navigation to `/docs/intro` without page reload

**Expected Behavior**:
- Focus indicator: 2px solid cyan ring with 4px offset
- Button accessible via keyboard
- Enter key triggers navigation
- No page reload (SPA navigation)

---

### T051: Mobile Device Testing
**Target**: iPhone 12 (390×844), Samsung Galaxy S21 (360×800)

**How to Test**:

**Option 1: Browser DevTools**
1. Open http://localhost:3000 in Chrome
2. Press F12 → Toggle Device Toolbar (Ctrl+Shift+M)
3. Select "iPhone 12 Pro" from device dropdown
4. Verify:
   - Hero takes full viewport height
   - 3D cube placeholder hidden
   - Stats cards stack vertically
   - Button fills width (max 280px)
   - Text readable at 390px width
5. Switch to "Galaxy S21" (360px width)
6. Verify same behavior

**Option 2: Real Devices**
1. Get local IP: `hostname -I`
2. Serve site: `npm run serve -- --host 0.0.0.0`
3. Access from mobile: http://[YOUR_IP]:3000
4. Test on physical iPhone 12 and Samsung Galaxy S21

**Expected Behavior**:
- Layout stacks vertically on mobile
- All text readable
- Button tap-friendly (60px height)
- No horizontal scrolling

---

### T052: Performance Validation
**Target**: FCP <1.5s, TTI <3s

**How to Test**:
1. Open http://localhost:3000 in Chrome
2. Press F12 → Performance tab
3. Click Record button (circle)
4. Reload page (Ctrl+R)
5. Stop recording after page loads
6. Check metrics in summary:
   - **FCP** (First Contentful Paint): Should be <1500ms
   - **TTI** (Time to Interactive): Should be <3000ms
   - **LCP** (Largest Contentful Paint): Should be <2500ms

**Expected Results**:
- FCP: <1.5s ✓
- TTI: <3s ✓
- Main thread work: <2s
- Bundle size: <500KB (excluding Three.js which is lazy-loaded)

---

### T053: Animation Performance
**Target**: 60fps animations on desktop with discrete GPU

**How to Test**:
1. Open http://localhost:3000 in Chrome on desktop
2. Verify discrete GPU is active:
   - Chrome: `chrome://gpu`
   - Look for "Graphics Feature Status"
3. Press F12 → Performance tab
4. Enable FPS meter: More tools → Rendering → Frame Rendering Stats
5. Scroll down to stats section
6. Observe FPS during:
   - Button pulse animation (3s interval)
   - Stats cards fade-in animation
   - Hover effects on stats cards

**Expected Results**:
- **FPS**: Solid 60fps during animations
- **Frame drops**: None or <2%
- **GPU-accelerated**: transform and opacity only
- **Main thread**: No long tasks during animations

**Performance Warnings**:
- If FPS drops below 30fps, check:
  - GPU acceleration enabled
  - Hardware acceleration in Chrome settings
  - CSS animations use only transform/opacity

---

## Test Results Summary

| Task | Status | Automated | Manual Required | Result |
|------|--------|-----------|-----------------|--------|
| T043 | ✅ | Yes | No | PASS (100vh verified) |
| T044 | ✅ | Yes | No | PASS (768px breakpoint) |
| T045 | ✅ | Yes | No | PASS (320px, 2560px) |
| T046 | ✅ | Yes | No | PASS (14.02:1 ratio) |
| T047 | ✅ | Yes | No | PASS (h1 hierarchy) |
| T048 | ✅ | Yes | No | PASS (light mode) |
| T049 | ⏳ | Partial | Yes | Lighthouse CI required |
| T050 | ⏳ | No | Yes | Browser test required |
| T051 | ⏳ | No | Yes | Device test required |
| T052 | ⏳ | No | Yes | DevTools required |
| T053 | ⏳ | No | Yes | FPS monitoring required |

---

## Quick Start Testing

```bash
# 1. Build the site
npm run build

# 2. Serve locally
npm run serve

# 3. Open browser and test
# - Keyboard navigation: Tab → Enter
# - Mobile: DevTools → Device Toolbar
# - Performance: DevTools → Performance tab
# - Lighthouse: DevTools → Lighthouse tab

# 4. Run Lighthouse CI (optional)
npx lhci autorun --url=http://localhost:3000
```

---

## Notes

- All automated verifications passed ✅
- Manual tests require browser environment
- Use Chrome DevTools for best testing experience
- Lighthouse CI provides comprehensive audit
- Tests T049-T053 should be run before deployment
