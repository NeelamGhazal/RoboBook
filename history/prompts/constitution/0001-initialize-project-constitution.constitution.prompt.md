---
id: 0001
title: Initialize project constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: developer
command: /sp.constitution
labels: ["governance", "hackathon", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
/sp.constitution # 🤖 PHYSICAL AI & HUMANOID ROBOTICS TEXTBOOK — OFFICIAL CONSTITUTION
*Hackathon I – Panaversity | Submission Deadline: Nov 30, 2025, 6:00 PM PKT*

## 🎯 PROJECT MISSION
Build an **AI-native, interactive technical textbook** on **Physical AI & Humanoid Robotics** using **Spec-Kit Plus** and **Claude Code**, deployed via **Docusaurus to GitHub Pages**, with:
- **21 structured chapters** (1 intro + 4 modules × 5 chapters)
- **Embedded RAG chatbot** (FastAPI + Qdrant + Neon Postgres + OpenAI)
- **User authentication** (Better-Auth with background survey)
- **Per-chapter personalization** (Simplified/Standard/Advanced)
- **Per-chapter Urdu translation** (Roman Urdu + RTL layout)

This project fulfills Panaversity's vision of next-gen **AI-authored educational content**.

---

## 📚 BOOK STRUCTURE & CONTENT

### Modules & Chapters (21 Total)
- **Intro**: Foundations of Physical AI, Embodied Intelligence, Course Roadmap
- **Module 1: Robotic Nervous System (ROS 2)**
  Nodes, Topics, Services, rclpy, URDF
- **Module 2: Digital Twin (Gazebo & Unity)**
  Physics simulation, sensor emulation (LiDAR, IMU, Depth Camera)
- **Module 3: AI-Robot Brain (NVIDIA Isaac™)**
  Isaac Sim, Isaac ROS, VSLAM, Nav2, Sim-to-Real
- **Module 4: Vision-Language-Action (VLA)**
  Whisper → voice, LLMs → ROS actions, Capstone: Autonomous Humanoid

### Chapter Template (Mandatory for All)
1. **Learning Objectives** (3–5 bullet points)
2. **Theory** (clear explanations + analogies)
3. **Code Examples** (Python 3.10+, ROS 2, PEP 8, tested on Ubuntu 22.04)
4. **Exercises** (discussion or implementation)
5. **Prerequisites** (stated at top)
6. **Word Count**: 2000–3000 words (excluding code)

### Diagrams & Media
- **Diagrams**: Mermaid only (no external images)
- **Code Blocks**: Show **input + expected output**
- **No decorative content** — every element must teach

---

## 🎨 UI/UX & DESIGN THEME

### Color Palette: **HUMARIDE ROBOTICS**
- **Primary (Neon Cyan)**: `#00F0FF`
- **Secondary (Magenta)**: `#FF2A6D`
- **Dark BG**: `#0A0A0F`
- **Light BG**: `#F8F9FB`
- **Card BG**: `#12131A` (dark), `#FFFFFF` (light)
- **Text**: `#D6D6D6` (dark), `#2A2A2A` (light)

### Landing Page (Custom Docusaurus Page at `/`)
- **Layout**: 50/50 split (desktop), stacked (mobile)
- **Left**: Animated **tech visualization** (Three.js rotating cube with neon edges, `#00F0FF`, 60fps)
- **Right**:
  - **Heading**: "Physical AI & Humanoid Robotics" (48px, cyan→magenta gradient)
  - **Subheading**: "From Digital Intelligence to Embodied Systems"
  - **"Read Book" Button**: 200×60px, hover glow (`box-shadow: 0 0 20px #00F0FF`)
- **Below**: 4 animated **stats cards** ("4 Modules", "21 Chapters", etc.) with **glass-morphism** and **hover glow**
- **Performance**: Load <3s, Lighthouse >90, fully responsive

### Book Reader Page (`/docs/*`)
- **Typography**:
  - Headings: **Inter/Poppins**
  - Body: **16–18px**, line-height **1.7**
  - Code: **Fira Code**
- **Layout**:
  - Clean TOC sidebar (collapsible)
  - Soft neon divider lines
  - **Smooth animations**: page fade-in, scroll-triggered reveals, chapter highlight
- **Code Blocks**: Modern theme, syntax-highlighted, with output

---

## ⚙️ TECHNICAL STACK

| Layer | Technology |
|------|------------|
| **Frontend** | Docusaurus 3.x, React, Tailwind + CSS Modules |
| **Animations** | Framer Motion, Three.js (landing only) |
| **Auth** | **Better-Auth** (email/password + user background survey) |
| **Backend** | **FastAPI** (chatbot + personalization API) |
| **Database** | **Neon Postgres** (users, preferences, chat history) |
| **Vector DB** | **Qdrant Cloud** (free tier) |
| **AI** | OpenAI API (gpt-4o-mini), ChatKit SDK |
| **Deployment** | GitHub Pages (frontend), Render/Vercel (backend) |

---

## 🎁 BONUS FEATURES (200 pts total)

| Feature | Requirement | Points |
|--------|-------------|--------|
| **1. Reusable Intelligence** | Create `/skill chapter-generator` and `/subagent chatbot-trainer` | +50 |
| **2. Better-Auth Signup** | Ask at signup:
- Software level (Beginner → Expert)
- Hardware level (None → Expert)
- Robotics experience (None → Real robots) | +50 |
| **3. Personalization Button** | At top of **every chapter** → regenerates content as **Simplified / Standard / Advanced** based on user profile | +50 |
| **4. Urdu Translation Button** | At top of **every chapter** → translates to **Roman Urdu**, preserves code blocks, supports **RTL layout** | +50 |

---

## 🧪 QUALITY & TESTING

- **All code** must be **copy-paste runnable** on **Ubuntu 22.04**
- **Mobile tested**: iPhone 12, Samsung Galaxy S21
- **Accessibility**: WCAG 2.1 AA compliant
- **Chatbot**: Answers **text-selection queries**, <2s response, cites sources
- **Demo Video**: **<90 seconds**, shows:
  - Landing page animation
  - Book navigation
  - Chatbot answering selected text
  - Auth + personalization + Urdu button

---

## 📤 SUBMISSION REQUIREMENTS

Submit via: https://forms.gle/CQsSEGM3GeCrL43c8
Include:
- ✅ Public GitHub repo
- ✅ Live GitHub Pages URL
- ✅ <90s demo video (YouTube/Drive)
- ✅ WhatsApp number

**Deadline**: Sunday, Nov 30, 2025, **6:00 PM PKT**
**Live Zoom**: 6:00 PM PKT (optional presentation)

---

## 🔁 WORKFLOW MANDATE

All development **MUST** follow **Spec-Kit Plus** workflow **per feature**:
```text
/sp.constitution → /sp.specify → /sp.clarify → /sp.plan → /sp.tasks → /sp.implement
```

## Response snapshot

Constitution successfully created at version 1.0.0 with 8 core principles:

1. **Educational Quality First** (NON-NEGOTIABLE) - Measurable learning objectives, tested code, 2000-3000 words per chapter
2. **Structured Content Architecture** - 21 chapters across 4 modules with mandatory template
3. **Code Correctness & Testability** - Python 3.10+, ROS 2, Ubuntu 22.04 verified code
4. **Visual Learning Through Diagrams** - Mermaid-only visualizations
5. **Performance & Accessibility** - Lighthouse >90, WCAG 2.1 AA compliance
6. **Brand Consistency** - HUMARIDE ROBOTICS theme with defined color palette
7. **User-Centric Features** - Better-Auth, personalization (Simplified/Standard/Advanced), Urdu translation
8. **Intelligent Chatbot** - RAG-based with <2s response, FastAPI + Qdrant + Neon Postgres

Additional sections defined:
- Content Standards (word counts, code standards, exercises, prerequisites)
- Technical Architecture (Docusaurus 3.x, FastAPI, Neon Postgres, Qdrant, OpenAI)
- UI/UX Requirements (responsive design, performance targets, animations, accessibility)
- Workflow & Process (Spec-Kit Plus mandatory workflow, PHR/ADR requirements)
- Governance (amendment process, compliance review, hackathon submission requirements)

## Outcome

- ✅ Impact: Complete constitution established with all hackathon requirements codified into 8 testable principles and 5 supporting sections. Version 1.0.0 ratified on 2025-12-09.
- 🧪 Tests: No automated tests (governance document). Manual validation: all placeholders filled, no bracketed tokens remaining, version semantics correct.
- 📁 Files: Modified `.specify/memory/constitution.md` (complete rewrite from template). Validated alignment with `.specify/templates/plan-template.md`, `spec-template.md`, and `tasks-template.md`.
- 🔁 Next prompts: Ready for `/sp.specify` to define first feature (likely landing page or chapter generation infrastructure).
- 🧠 Reflection: Constitution captures all hackathon constraints (21 chapters, RAG chatbot, Better-Auth, personalization, Urdu translation, HUMARIDE theme, performance targets). Foundation is solid for feature development.

## Evaluation notes (flywheel)

- Failure modes observed: None. Template transformation successful with all placeholders resolved.
- Graders run and results (PASS/FAIL): Manual validation PASS - no unresolved tokens, version format correct, all sections coherent.
- Prompt variant (if applicable): Standard constitution creation from detailed requirements.
- Next experiment (smallest change to try): Begin feature specification with `/sp.specify` for landing page or chapter infrastructure, whichever user prioritizes.
