---
id: 0003
title: Add theme toggle and detailed specs
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 003-ocean-sapphire-theme
user: user
command: /sp.constitution
labels: ["theme-toggle", "navbar-specs", "typography", "performance", "restrictions"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

## Prompt

XX. Theme Toggle (Mandatory UI Control)

RULE: The landing page must include a Light/Dark theme toggle located in the navbar.

Requirements

Default theme = Dark (Ocean Sapphire)

Toggle switches between:

Dark Theme (existing Ocean Sapphire palette)

Light Theme (color-inverted Ocean Sapphire palette)

Only colors change — layout, spacing, typography, animation same rehte hain

Toggle reacts via React state (no localStorage)

Transition animation: 200ms ease-in-out

Must remain WCAG AA contrast-compliant in both modes

Light Theme Palette

Background: #f3f8ff

Text: #001529

Accent Cyan: #0096ff

Borders: rgba(0, 150, 255, 0.3)

Cards: rgba(255, 255, 255, 0.6) with blur

Placement

Navbar → Right side, after EN/UR language toggle

Rationale: Provides user-controlled accessibility/style preference while maintaining brand identity.

[Additional comprehensive requirements for Project Identity, File Modifications, Navbar Requirements, Typography Rules, Hero Section Requirements, Feature Section Requirements, Design System, Performance Rules, and Non-Negotiable Restrictions]

## Response snapshot

Successfully updated constitution to version 1.4.0 with comprehensive additions:

1. **Theme Toggle (Section XX)**: Added mandatory Light/Dark theme toggle with React state, 200ms transitions, WCAG AA compliance, and specific light theme palette.

2. **File Modification Restrictions (Section VI)**: Explicitly defined allowed files (index.tsx, Hero/*, FeatureCards/*, custom.css) and forbidden modifications (routing, docs/blog, backend, translation systems).

3. **Navbar Specifications**: Added precise dimensions (72px height, 40×40px icon, 22px Orbitron logo text, 17px menu items, 32px gaps, 28px control icons, 24px spacing).

4. **Typography Rules**: Clarified Orbitron usage limited to landing page only (navbar, headings, CTAs). Specified exact font sizes: Hero heading (4.5rem), Hero paragraph (1.3rem), Button text (1.2rem), Section heading (2.8rem), Card titles (1.5rem), Card text (1rem).

5. **Hero Section Details**: Added spacing requirements (100px top padding, 24px side padding, 20px heading-paragraph gap, 28px paragraph-button gap), precise image dimensions (520-580px width, auto height, WebP format).

6. **Feature Section Specifications**: Defined layout (1200px max-width, 80px vertical padding, 24px desktop / 16px mobile card gaps), card dimensions (220px min height, 12px border-radius, 24px padding, 40/60 image/text split).

7. **Performance Standards (Section XXI)**: Image optimization requirements (WebP format, robot max 200KB, cards max 50KB each), no layout shift (CLS = 0), lazy loading.

8. **Non-Negotiable Restrictions (Section XXII)**: Explicitly forbidden actions (new pages, backend logic, translation systems, chatbot, routing changes, docs/blog font changes).

Constitution version bumped from 1.3.0 to 1.4.0 (MINOR bump - new principles and materially expanded guidance).

## Outcome

- ✅ Impact: Constitution updated with comprehensive theme toggle specifications, precise navbar/typography/layout requirements, strict file modification boundaries, and performance standards
- 🧪 Tests: No tests (documentation update)
- 📁 Files: .specify/memory/constitution.md (updated)
- 🔁 Next prompts: Implement theme toggle component, add light theme CSS variables, update navbar dimensions, optimize images to WebP format
- 🧠 Reflection: Comprehensive specifications prevent ambiguity and ensure consistent implementation. Theme toggle adds user control while maintaining brand identity.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (constitution update)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement theme toggle as isolated component first, then integrate into navbar
