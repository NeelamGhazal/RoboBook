---
id: 1
title: Landing Page Redesign
stage: general
date: 2025-12-10
model: claude-sonnet-4-5-20250929
feature: none
branch: 003-ocean-sapphire-theme
user: claude
command: sp.constitution
labels: ["landing-page", "redesign", "ui-ux", "navbar", "hero", "feature-cards"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/docusaurus.config.ts
  - website/src/css/custom.css
  - website/src/pages/index.tsx
  - website/src/components/Hero/Hero.tsx
  - website/src/components/FeatureCards/FeatureCard.tsx
  - website/src/components/FeatureCards/FeatureCards.tsx
  - .specify/memory/constitution.md
tests: []
---

# Landing Page Redesign

## Prompt Text

FINAL PROMPT — LANDING PAGE RESTRUCTURE + NAVBAR + NEW HERO + FEATURE CARDS + FONT + IMAGE FIX
TASK:

Redesign the landing page structure while preserving overall site functionality.
Modify only the specified files.

✅ SCOPE — YOU MAY MODIFY ONLY THESE FILES
src/pages/index.tsx
src/components/Hero/*       (hero section components only)
src/components/FeatureCards/*
src/css/custom.css

❌ DO NOT MODIFY

Any animation code (except replacing humanoid animation with static image)

Blog/docs folders

Other pages/routes

Docusaurus root config (except navbar items)

Build/deployment configuration

─────────────────────────────────────────────
SECTION 1 — NAVBAR CHANGES
─────────────────────────────────────────────
✅ A. Replace Logo

Remove:

Default Docusaurus logo

Add:

A clean, minimal, robotic tech icon (use any SVG available in project or simple robot head icon)

Display text: RoboBook

Specs:

Logo container: 200×60px

Icon: 40×40px

Text: Orbitron font
(You MUST import Orbitron into the site and apply it to this.)

✅ B. Add Language Toggle (Right Side)

Place it: Right side of navbar → after nav items → before GitHub icon

Requirements:

UI Style: Simple icon + dropdown OR a 2-state toggle switch

States:

English (default)

Urdu

Only visual toggle — no translation functionality needed

Implement using Docusaurus navbar.items

─────────────────────────────────────────────
SECTION 2 — HERO SECTION RESTRUCTURE
─────────────────────────────────────────────
❗ IMPORTANT FIXES
❌ REMOVE COMPLETELY:

Any humanoid / "HUMARIDE" wording

Any humanoid walking animation canvas

✅ ADD INSTEAD:

A professional, elegant tech-style static robot image on the right side
(PNG/SVG inside /static/img/)

NEW LAYOUT (MANDATORY)
Desktop (≥1024px):

Left 50% — Right 50%

Tablet (768–1023px):

Left 60% — Right 40%

Mobile (<768px):

Stack vertically → text then image

LEFT SIDE CONTENT

(All text must use Orbitron font)

Heading:

Physical AI & Humanoid Robotics

Font: Orbitron

Size: 4.5rem

Line height: 1.2

Paragraph:

Learn to control physical androids using ROS 2 and Isaac Sim—even if you only have a laptop.

Font: Orbitron Italic (or fallback italic)

Size: 1.3rem

Button:

Start Learning

200×60px

Cyan border

Hover: cyan glow + slight scale

Rounded corners consistent with theme

RIGHT SIDE CONTENT

Replace complex robot animation

Add clean static robot/tech illustration image

Responsive: must shrink correctly

No animation JS required

─────────────────────────────────────────────
SECTION 3 — NEW FEATURE SECTION
─────────────────────────────────────────────
Insert this below Hero Section, before any other content.
Heading

What's Inside This Book?

Orbitron

Georgia SERIF styling NOT required anymore unless needed

Size: 2.8rem

Centered

Margin-bottom: 6rem

Description

This book guides you step-by-step through building a complete Physical AI project—from writing a spec-driven book to embedding an intelligent RAG chatbot, personalizing content, and integrating humanoid robotics simulations.

Centered

Max-width: 900px

Size: 1.1rem

mb-16

FEATURE CARDS — EXACT STRUCTURE
Card container:

Flex row

Left: Image (40%)

Right: Text (60%)

Aspect ratio: 3:2

Glassmorphism: use existing theme classes

Border radius: 12px

Hover: 3s shimmer gradient sweep

Gap between cards: 24px

Mobile: Stack vertically (image → text)

CARD CONTENT
Card 1

Title: AI/Spec-Driven Book Creation
Text: Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus.
Image: AI/tech illustration

Card 2

Title: Integrated RAG Chatbot
Text: Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres.
Image: Chatbot/AI agent illustration

Card 3

Title: Personalization & Translations
Text: Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter.
Image: Globe/translation-themed illustration

─────────────────────────────────────────────
SECTION 4 — DESIGN SYSTEM REQUIREMENTS
─────────────────────────────────────────────
Font:

Entire landing page (navbar + hero + features) MUST use Orbitron

Import via Google Fonts or local file

Replace old fonts where needed

DO NOT USE ANY "HUMARIDE" TEXT ANYWHERE

Theme:

Keep Ocean Sapphire palette

Keep glow effects

Keep glassmorphism from existing CSS

Animations:

Must respect prefers-reduced-motion

No complex JS animations needed for robot image

─────────────────────────────────────────────
ACCEPTANCE CRITERIA
─────────────────────────────────────────────

✓ Navbar shows robot icon + "RoboBook" with Orbitron font
✓ Language toggle appears & works visually
✓ Hero layout is perfectly split (text left, image right)
✓ Humanoid animation removed & replaced with static robot image
✓ ZERO usage of "Humaride" anywhere
✓ All hero text rendered in Orbitron
✓ Feature section with 3 cards appears correctly
✓ Glassmorphism hover & shimmer works
✓ Fully responsive 320px → 2560px
✓ Build passes with no errors

## Response Text

Successfully redesigned the landing page structure according to the requirements. Key changes made:

1. Updated the project constitution to reflect new requirements (changed typography from Georgia to Orbitron, updated branding from "HUMARIDE" to "RoboBook")

2. Updated the navbar in docusaurus.config.ts:
   - Changed title to "RoboBook"
   - Added Orbitron font import
   - Added language toggle for English/Urdu

3. Created new Hero component with:
   - 50/50 split layout (desktop) / 60/40 (tablet) / stacked (mobile)
   - Orbitron typography for all text
   - Static robot image replacing the animation
   - "Start Learning" CTA button with cyan border and hover effects

4. Created new FeatureCards components with:
   - 3 cards with flex row layout (left image 40%, right text 60%)
   - Glassmorphism styling with shimmer hover effect
   - Proper content for AI/Spec-Driven Book Creation, RAG Chatbot, and Personalization

5. Updated CSS styling to:
   - Use Orbitron font throughout
   - Implement responsive layouts
   - Add shimmer animations for feature cards
   - Maintain Ocean Sapphire theme

6. Successfully built the website with no errors.

The landing page now meets all requirements: responsive design, Orbitron typography, glassmorphism cards with shimmer effects, static robot image instead of animation, and language toggle.