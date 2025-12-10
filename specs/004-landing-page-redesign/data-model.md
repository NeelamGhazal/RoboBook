# Data Model: RoboBook Landing Page Redesign

**Feature**: 004-landing-page-redesign
**Date**: 2025-12-10
**Status**: Complete

## Overview

This document defines the data structures and state management for the RoboBook landing page redesign. Since this is a frontend-only static site, the "data model" consists of React component state, props interfaces, and configuration objects rather than database schemas.

---

## Entity 1: Theme State

**Purpose**: Manages light/dark theme toggle state for the landing page

**Type**: React State (useState hook)

**Schema**:
```typescript
type ThemeMode = 'dark' | 'light';

interface ThemeState {
  mode: ThemeMode;
}
```

**Default Value**: `'dark'` (per Constitution §XX)

**State Management**:
- Location: `src/pages/index.tsx` or root layout component
- Hook: `const [theme, setTheme] = useState<ThemeMode>('dark')`
- Persistence: None (resets to dark on page reload, per Constitution §XX - no localStorage)

**State Transitions**:
```
dark -> light  (user clicks theme toggle)
light -> dark  (user clicks theme toggle)
```

**Validation Rules**:
- Value must be exactly 'dark' or 'light'
- State change triggers CSS variable update via data-theme attribute
- Transition animation: 200ms ease-in-out (enforced via CSS)

**Related Components**:
- Navbar (contains theme toggle button)
- All styled components (consume theme via CSS variables)

---

## Entity 2: Hero Section Content

**Purpose**: Defines text content and image source for the hero section

**Type**: Static Configuration Object

**Schema**:
```typescript
interface HeroContent {
  heading: string;
  paragraph: string;
  buttonText: string;
  buttonLink: string;
  robotImage: {
    src: string;
    alt: string;
    width: number;
    height: number;
  };
}
```

**Default Value**:
```typescript
const heroContent: HeroContent = {
  heading: 'Physical AI & Humanoid Robotics',
  paragraph: 'Learn to control physical androids using ROS 2 and Isaac Sim—even if you only have a laptop',
  buttonText: 'Start Learning',
  buttonLink: '/docs/intro',
  robotImage: {
    src: '/img/robot-halfbody.webp',
    alt: 'Professional 3D half-body robot demonstrating humanoid robotics',
    width: 550,
    height: 733,
  },
};
```

**Validation Rules**:
- Heading: 3-60 characters, Orbitron font at 4.5rem
- Paragraph: 50-200 characters, Georgia serif at 1.3rem
- Button text: 5-25 characters
- Robot image: WebP format, <200KB, 520-580px width
- Image alt text: Descriptive, 50-150 characters

**Related Components**:
- Hero.tsx (container)
- HeroText.tsx (heading, paragraph, button)
- HeroImage.tsx (robot image)

---

## Entity 3: Feature Card

**Purpose**: Represents a single feature card with image, title, and description

**Type**: TypeScript Interface for Component Props

**Schema**:
```typescript
interface FeatureCard {
  id: string;
  title: string;
  description: string;
  image: {
    src: string;
    alt: string;
    width: number;
    height: number;
  };
}
```

**Example Instance**:
```typescript
const featureCard: FeatureCard = {
  id: 'ai-book',
  title: 'AI/Spec-Driven Book Creation',
  description: 'Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus.',
  image: {
    src: '/img/feature-ai-book.webp',
    alt: 'AI-powered book creation with code and documentation',
    width: 540,
    height: 360,
  },
};
```

**Validation Rules**:
- ID: Unique identifier, kebab-case, 5-30 characters
- Title: 10-50 characters, Orbitron font at 1.5rem
- Description: 50-150 characters, Georgia serif at 1rem
- Image: 3:2 aspect ratio (e.g., 540×360px), WebP format, <50KB
- Image alt text: Descriptive, 50-150 characters

**Relationships**:
- Collection: Array of 3 FeatureCard instances
- Parent: FeatureCards component
- Child: FeatureCard component (renders single card)

---

## Entity 4: Feature Cards Collection

**Purpose**: Array of all feature cards to be displayed on the landing page

**Type**: Static Configuration Array

**Schema**:
```typescript
type FeatureCardsCollection = FeatureCard[];
```

**Default Value**:
```typescript
const featuresData: FeatureCardsCollection = [
  {
    id: 'ai-book',
    title: 'AI/Spec-Driven Book Creation',
    description: 'Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus.',
    image: {
      src: '/img/feature-ai-book.webp',
      alt: 'AI-powered book creation with code and documentation',
      width: 540,
      height: 360,
    },
  },
  {
    id: 'rag-chatbot',
    title: 'Integrated RAG Chatbot',
    description: 'Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres.',
    image: {
      src: '/img/feature-chatbot.webp',
      alt: 'RAG chatbot interface with AI brain and database connections',
      width: 540,
      height: 360,
    },
  },
  {
    id: 'personalization',
    title: 'Personalization & Translations',
    description: 'Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter.',
    image: {
      src: '/img/feature-personalization.webp',
      alt: 'Globalization with language symbols and user profiles',
      width: 540,
      height: 360,
    },
  },
];
```

**Validation Rules**:
- Length: Exactly 3 cards (per Constitution §VIII)
- Order: Displayed in array order
- Uniqueness: All IDs must be unique

**Related Components**:
- FeatureCards.tsx (maps over array to render cards)
- FeatureCard.tsx (renders individual card)

---

## Entity 5: Navbar Configuration

**Purpose**: Defines navbar menu items, logo, and control buttons

**Type**: Static Configuration Object

**Schema**:
```typescript
interface NavbarConfig {
  logo: {
    icon: string;
    text: string;
    link: string;
  };
  menuItems: Array<{
    label: string;
    link: string;
  }>;
  controls: {
    languageToggle: {
      options: ['EN', 'UR'];
      default: 'EN';
    };
    themeToggle: boolean;
    githubLink: string;
  };
}
```

**Default Value**:
```typescript
const navbarConfig: NavbarConfig = {
  logo: {
    icon: '/img/robobook-icon.svg',
    text: 'RoboBook',
    link: '/',
  },
  menuItems: [
    { label: 'Textbook', link: '/docs/intro' },
    { label: 'Blog', link: '/blog' },
  ],
  controls: {
    languageToggle: {
      options: ['EN', 'UR'],
      default: 'EN',
    },
    themeToggle: true,
    githubLink: 'https://github.com/panaversity/robobook',
  },
};
```

**Validation Rules**:
- Logo icon: 40×40px, SVG or PNG
- Logo text: "RoboBook", Orbitron 22px, 700 weight
- Menu items: 1-5 items, labels 5-15 characters
- Language toggle: Visual only (no functional translation per Constitution §XXII)
- GitHub link: Valid URL

**State**:
- Language selection: Local state (visual only, not persisted)
- Theme: Global state (managed by ThemeState entity)

**Related Components**:
- Navbar.tsx

---

## CSS Variables (Theme Palette)

**Purpose**: Define color tokens for light and dark themes

**Type**: CSS Custom Properties

**Dark Theme** (default):
```css
:root[data-theme='dark'] {
  --bg-primary: #001529;
  --bg-secondary: #002140;
  --text-primary: #b8d4ff;
  --text-secondary: #80c2ff;
  --accent-cyan: #0096ff;
  --accent-bright: #00d4ff;
  --card-bg: rgba(0, 50, 100, 0.3);
  --border-color: rgba(0, 150, 255, 0.3);
  --shimmer-color: rgba(0, 150, 255, 0.1);
}
```

**Light Theme**:
```css
:root[data-theme='light'] {
  --bg-primary: #f3f8ff;
  --bg-secondary: #ffffff;
  --text-primary: #001529;
  --text-secondary: #003a6d;
  --accent-cyan: #0096ff;
  --accent-bright: #0077cc;
  --card-bg: rgba(255, 255, 255, 0.6);
  --border-color: rgba(0, 150, 255, 0.3);
  --shimmer-color: rgba(0, 150, 255, 0.1);
}
```

**Usage**:
All styled components reference these variables via `var(--bg-primary)`, etc.

**Validation**:
- WCAG AA contrast ratios: Dark theme ≥10:1, Light theme ≥16:1 (verified)
- Transition: 200ms ease-in-out on all color properties

---

## Component Prop Interfaces

### HeroProps
```typescript
interface HeroProps {
  content: HeroContent;
  theme: ThemeMode;
}
```

### FeatureCardProps
```typescript
interface FeatureCardProps {
  card: FeatureCard;
  onHover?: () => void;
}
```

### FeatureCardsProps
```typescript
interface FeatureCardsProps {
  cards: FeatureCardsCollection;
}
```

### NavbarProps
```typescript
interface NavbarProps {
  config: NavbarConfig;
  theme: ThemeMode;
  onThemeToggle: () => void;
}
```

---

## State Management Summary

| Entity | Storage | Persistence | Scope |
|--------|---------|-------------|-------|
| Theme State | React useState | None (resets to dark) | Page-level |
| Hero Content | Static object | N/A | Component-level |
| Feature Cards | Static array | N/A | Component-level |
| Navbar Config | Static object | N/A | Component-level |
| Language Selection | Local state | None (visual only) | Navbar component |

**No external state management** (Redux, MobX, Context API) required. Simple React state sufficient for theme toggle.

**No API calls or data fetching**. All content is static and defined at build time.

**No localStorage or cookies**. Theme state resets on page reload per Constitution §XX.

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      index.tsx (Root)                        │
│  - useState<ThemeMode>                                       │
│  - useEffect: document.documentElement.setAttribute          │
└────────────────┬────────────────────────────────────────────┘
                 │
      ┌──────────┼──────────┬──────────────────────┐
      │          │           │                      │
      ▼          ▼           ▼                      ▼
┌─────────┐ ┌────────┐ ┌────────────┐  ┌──────────────────┐
│ Navbar  │ │  Hero  │ │  Features  │  │  CSS Variables   │
│         │ │        │ │            │  │  :root[data-     │
│ Props:  │ │ Props: │ │ Props:     │  │   theme=...]     │
│ - theme │ │ - cont │ │ - cards[]  │  │                  │
│ - onTog │ │ - theme│ │            │  │ Consumes:        │
└─────────┘ └────┬───┘ └──────┬─────┘  │ - theme state    │
                 │             │        │   via attribute  │
           ┌─────┴──┐     ┌───┴─────┐  └──────────────────┘
           ▼        ▼     ▼         ▼
       HeroText HeroImg  FeatureCard (×3)
```

**Data Flow**:
1. User clicks theme toggle → `setTheme('light')` called
2. useEffect updates `document.documentElement.setAttribute('data-theme', 'light')`
3. CSS variables switch from `:root[data-theme='dark']` to `:root[data-theme='light']`
4. All components re-render with new colors (200ms transition)

---

## Validation & Constraints

### Type Safety
- All entities use TypeScript interfaces
- Props validated at compile time
- No `any` types allowed

### Runtime Validation
- Theme state: Must be 'dark' or 'light' (enforced by TypeScript literal type)
- Image paths: Must exist in `static/img/` directory
- Link URLs: Must be valid internal or external URLs

### Performance Constraints
- Robot image: <200KB
- Feature card images: <50KB each
- Total images: <350KB
- No lazy loading overhead (native browser feature)

### Accessibility Constraints
- All images: Must have descriptive alt text
- Theme toggle: Must be keyboard accessible
- Contrast ratios: Must meet WCAG AA (≥4.5:1)

---

## Summary

This data model defines:
- **1 state entity** (Theme State) managed by React useState
- **4 configuration entities** (Hero Content, Feature Cards, Navbar Config, CSS Variables) as static objects/arrays
- **4 TypeScript interfaces** for component props
- **No persistent storage** (no database, localStorage, or cookies)
- **No API contracts** (frontend-only static site)

All entities align with Constitution v1.4.0 specifications and support the requirements defined in spec.md.
