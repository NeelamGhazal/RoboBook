---
id: 10
title: update-constitution-ocean-sapphire-ui-ux
stage: constitution
date: '2025-12-10'
model: Claude Sonnet 4.5
feature: 003-ocean-sapphire-theme
branch: 003-ocean-sapphire-theme
user: Claude
command: /sp.constitution
labels:
  - constitution
  - ui-ux
  - ocean-sapphire
  - design-system
files:
  - .specify/memory/constitution.md
tests: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
---

# Update Constitution with Ocean Sapphire UI/UX Requirements

## User Input

```
# 🌊 Physical AI & Humanoid Robotics — Ocean Sapphire Constitution

## GOVERNING PRINCIPLES

### 1. UI/UX Design System
- **Color Palette (Ocean Sapphire)**:
  - Primary Dark: `#001529`
  - Primary Mid: `#002140`
  - Accent Cyan: `#0096ff`
  - Bright Cyan: `#00d4ff`
  - Light Cyan: `#80c2ff`
  - Pale Text: `#b8d4ff`
- **Typography**: Georgia serif for all headings; body in readable sans-serif
- **Layout**: Mobile-first, responsive grid system
- **Accessibility**: WCAG AA compliant contrast; all animations respect `prefers-reduced-motion`

### 2. Mandatory Page Structure

#### Navbar (Fixed Top, Backdrop Blur)
- **Left**: Logo only (`200×60px`, text: "RoboBook", no extra branding)
- **Right**: Navigation items → `Textbook | Blog | EN/UR Toggle | Theme Toggle | GitHub Icon`
- **Interaction**: Logo hover → subtle cyan glow (`box-shadow: 0 0 15px rgba(0, 150, 255, 0.4)`)

#### Hero Section (Responsive Grid)
- **Left (Text)**:
  - H1: `"Physical AI & Humanoid Robotics"` (Georgia, 4.5em, white)
  - Subheading: `"Learn to control physical androides using ROS 2 and Isaac Sim—even if you only have a laptop"` (Georgia italic, 1.3em, `#80c2ff`)
  - CTA Button: `"Start Learning"` (200×60px, `#0096ff` border, hover glow + scale)
- **Right (Animation)**:
  - **Canvas**: `600×600px` minimum
  - **Content**: Professional humanoid wireframe with:
    - Clear head, torso, 2 arms, 2 legs
    - Animated walking cycle (arms swing alternately, legs move)
    - Neural network: 30+ radiating cyan lines from head (pulsing)
    - Particle system: 150 particles forming robot silhouette
    - Glow: cyan gradient (`#00d4ff`) with smooth 60fps rendering

#### Features Section
- **Heading**: `"What's Inside This Book?"` (Georgia, 2.8em, white)
- **Description**: `"A hands-on journey from ROS 2 fundamentals to autonomous humanoids with VLA models—designed for learners with basic Python knowledge."` (`#b8d4ff`, 1.1em)

#### Feature Cards (4 Items)
- **Shape**: **Rectangle only** (aspect ratio **3:2**, **NOT square or cube**)
- **Layout**:
  - **Left**: Icon (`60×60px`, lucide-react: Cpu, Globe, Zap, Book)
  - **Right**:
    - Heading (`#00d4ff`, 1.2em)
    - Description (`#b8d4ff`, 0.95em)
- **Style**:
  - Glassmorphism: `background: rgba(0, 50, 100, 0.3)`, `backdrop-filter: blur(15px)`, `border-radius: 12px`
  - **Card Content**:
    1. **"ROS 2 Fundamentals"** – "Master robot communication & control systems"
    2. **"Isaac Sim Integration"** – "Simulate humanoids in NVIDIA's physics engine"
    3. **"VLA Models"** – "Implement vision-language-action AI controllers"
    4. **"Real Hardware"** – "Deploy to physical robots with confidence"
- **Hover**: Shimmer animation (cyan gradient sweep left→right, **3s duration**)

### 3. Technical Constraints
- **Language Toggle**: EN/UR in React state; Urdu = RTL alignment
- **No localStorage**: All state in React only
- **Performance**: Canvas optimized for 60fps; page load <2s
- **Framework**: Docusaurus + React + Tailwind CSS
- **Icons**: lucide-react only

### 4. Quality Standards
- **Color Accuracy**: Strict adherence to Ocean Sapphire hex values
- **Responsiveness**: Breakpoints at 1024px (desktop), 768px (tablet), 640px (mobile)
- **Animation Smoothness**: All hover effects = 300ms; shimmer = 3s infinite
- **Visual Fidelity**: Robot wireframe must be clearly humanoid with joints, fingers, and antenna
```

## Response

Updated the project constitution with comprehensive Ocean Sapphire UI/UX requirements. Added new sections VII-X for UI/UX Design System, Mandatory Page Structure, Technical Constraints, and Quality Standards. Updated section numbering throughout the document and incremented version from 1.1.0 to 1.2.0. The constitution now includes detailed specifications for navbar, hero section, feature cards, and technical requirements.

## Outcome

The constitution has been successfully updated with detailed UI/UX requirements for the Ocean Sapphire theme, providing clear implementation guidance for developers.