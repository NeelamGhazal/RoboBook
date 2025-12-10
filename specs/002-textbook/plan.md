# Implementation Plan: AI-Native Textbook — Physical AI & Humanoid Robotics

**Branch**: `002-textbook` | **Date**: 2025-12-09 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive 21-chapter Docusaurus textbook on Physical AI & Humanoid Robotics, organized into 4 modules: (1) ROS 2 fundamentals, (2) Gazebo/Unity simulation, (3) NVIDIA Isaac AI integration, (4) Vision-Language-Action capstone. Each chapter includes learning objectives, theory with analogies, Mermaid diagrams, executable Python code examples with expected output, and hands-on exercises. All code must run on Ubuntu 22.04 with ROS 2 Humble and Python 3.10+, following PEP 8 standards. Chapters contain 2000-3000 words of educational content, ensuring consistency through standardized templates and automated validation.

## Technical Context

**Language/Version**: Python 3.10+ (for code examples in chapters), Markdown (for content), Mermaid (for diagrams)
**Primary Dependencies**: Docusaurus 3.x (publishing platform), ROS 2 Humble (robotics framework for examples), Gazebo Fortress, Unity 2021.3 LTS, NVIDIA Isaac Sim, OpenAI Whisper, LLM access (OpenAI API or Ollama/LLaMA)
**Storage**: Git repository for markdown files, static file hosting for Docusaurus build output (GitHub Pages)
**Testing**: Manual content review, automated Docusaurus build validation (broken links, Mermaid syntax), Python code execution tests in Docker container (Ubuntu 22.04 + ROS 2 Humble)
**Target Platform**: Web browser (desktop and mobile) via Docusaurus static site, learner development environment (Ubuntu 22.04 for running code examples)
**Project Type**: Documentation/content project (not traditional software - creates educational markdown files, not application code)
**Performance Goals**: Docusaurus build time <5 minutes for 21 chapters, page load time <2 seconds, all code examples execute <30 seconds, learners complete Module 1 in 10 hours
**Constraints**: -1000 words per chapter (content quality), 100% code executability on Ubuntu 22.04 + ROS 2 Humble (reproducibility), Mermaid diagrams must render without build errors (visual clarity), PEP 8 compliance (code readability)
**Scale/Scope**: 21 markdown files (~50,000 words total), 4 modules with 5 chapters each plus 1 intro, 50+ Python code examples, 20+ Mermaid diagrams, 100+ exercises

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Educational Quality First ✅ APPLICABLE

**Status**: PASS (design will address)

- [x] 3-5 learning objectives per chapter (spec FR-013)
- [x] Theory with practical analogies (spec FR-002 section 4)
- [x] All code copy-paste runnable on Ubuntu 22.04 + Python 3.10+ (spec FR-003)
- [x] 2000–3000 words per chapter excluding code (spec FR-004)
- [x] Exercises requiring active engagement (spec FR-014)
- [x] Prerequisites stated at chapter top (spec FR-012)

**Plan Impact**: Phase 1 will create chapter template enforcing all 6 requirements. quickstart.md will include sample chapter demonstrating compliance.

### Principle II: Structured Content Architecture ✅ APPLICABLE

**Status**: PASS (spec defines structure)

- [x] 21 chapters across 4 modules (spec FR-001)
- [x] Introduction + Module 1 (ROS 2) + Module 2 (Gazebo/Unity) + Module 3 (Isaac) + Module 4 (VLA) (spec FR-006 through FR-009)
- [x] Mandatory template adherence: learning objectives, theory, code, exercises, prerequisites (spec FR-002)
- [x] Docusaurus sidebar organization (spec FR-005)

**Plan Impact**: data-model.md will define Chapter entity with all mandatory fields. Sidebar configuration included in quickstart.md.

### Principle III: Code Correctness & Testability ✅ APPLICABLE

**Status**: PASS (validation required)

- [x] Python 3.10+ with PEP 8 compliance (spec FR-003, Assumption 3)
- [x] ROS 2 Humble (spec FR-003, Assumption 2)
- [x] Tested on Ubuntu 22.04 LTS (spec FR-003, Assumption 1)
- [x] Complete setup in code blocks (spec FR-002 section 6)
- [x] Expected output shown (spec FR-010)
- [ ] **ACTION REQUIRED**: Automated testing strategy needed

**Plan Impact**: research.md will document Docker-based validation approach (Ubuntu 22.04 + ROS 2 Humble container). quickstart.md includes code validation checklist.

### Principle IV: Visual Learning Through Diagrams ✅ APPLICABLE

**Status**: PASS (spec requires Mermaid)

- [x] Mermaid diagrams exclusively (spec FR-011, Assumption 9)
- [x] System architectures, data flows, state machines (spec FR-002 section 5)
- [x] ROS 2 node communication graphs (implied by Module 1 content)
- [x] No decorative images (spec emphasizes functional diagrams)
- [x] Mermaid validation during Docusaurus build (spec FR-011)

**Plan Impact**: research.md will document Mermaid best practices for robotics diagrams. quickstart.md includes sample diagrams for common patterns (pub/sub, state machines).

### Principle V: Performance & Accessibility ⚠️ PARTIAL (Docusaurus config only)

**Status**: PASS (landing page feature addresses this)

- [x] Landing page already implements Lighthouse >90, WCAG 2.1 AA (001-landing-page feature)
- [x] Mobile responsive, keyboard navigation, heading hierarchy (001-landing-page feature)
- [x] Docusaurus default theme meets accessibility standards
- [ ] **NOTE**: Textbook content creation (this feature) inherits Docusaurus accessibility; no additional work required

**Plan Impact**: No additional planning needed. Docusaurus configuration from 001-landing-page ensures compliance.

### Principle VI: Brand Consistency ⚠️ PARTIAL (landing page handles UI)

**Status**: PASS (theme already applied)

- [x] Humaride Robotics color palette applied in 001-landing-page (custom.css, tailwind.config.js)
- [x] Fonts (Inter, Poppins) configured in 001-landing-page (docusaurus.config.ts)
- [x] Glass-morphism, neon accents on landing page
- [ ] **NOTE**: Textbook markdown content uses Docusaurus default styling; brand theme applied site-wide via existing configuration

**Plan Impact**: No additional planning needed. Content inherits brand theme from 001-landing-page configuration.

### Principle VII: User-Centric Features ❌ OUT OF SCOPE

**Status**: NOT APPLICABLE (explicitly excluded in spec.md)

- [ ] Better-Auth authentication (spec Out of Scope: "User authentication")
- [ ] User profiling and surveys (spec Out of Scope: "per-user progress tracking and personalization")
- [ ] Personalization buttons (spec Out of Scope: "Simplified/Standard/Advanced")
- [ ] Urdu translation (spec Out of Scope: "Urdu translation")

**Rationale**: Spec states "initial release is English-only" and "personalization (Simplified/Standard/Advanced) are future features". This feature focuses solely on creating 21 English chapters with standard technical depth.

**Plan Impact**: No work for this principle. Future feature will add personalization layer.

### Principle VIII: Intelligent Chatbot ❌ OUT OF SCOPE

**Status**: NOT APPLICABLE (explicitly excluded in spec.md)

- [ ] RAG chatbot (spec Out of Scope: "RAG chatbot integration")
- [ ] FastAPI backend (not part of textbook content creation)
- [ ] Qdrant vector DB (separate feature)
- [ ] Neon Postgres (separate feature)

**Rationale**: Spec states "The AI-powered Q&A chatbot is a separate feature (not part of this textbook content creation spec)".

**Plan Impact**: No work for this principle. Textbook content will be ingested by chatbot feature later.

### Constitution Check Summary

**Applicable Principles**: 4/8 (I, II, III, IV)
**Passing**: 4/4 ✅
**Partial (inherited)**: 2/8 (V, VI) ✅
**Out of Scope**: 2/8 (VII, VIII) ✅

**GATE RESULT**: ✅ PASS - Proceed to Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/002-textbook/
├── spec.md              # Feature specification (/sp.specify output)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command) - Chapter structure
├── quickstart.md        # Phase 1 output (/sp.plan command) - Chapter template
├── contracts/           # Phase 1 output (/sp.plan command) - Chapter schemas
│   └── chapter-schema.md
├── checklists/          # Quality validation
│   └── requirements.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Textbook Content (repository root - created during implementation)

**IMPORTANT**: This is a content creation project, not traditional software. The "source code" is 21 markdown chapters, not application code.

```text
website/docs/
├── intro.md                                    # Introduction chapter (P1 MVP)
│
├── module-1-ros2/                              # Module 1: The Robotic Nervous System
│   ├── chapter-1-nodes-architecture.md
│   ├── chapter-2-topics-pubsub.md
│   ├── chapter-3-services-clients.md
│   ├── chapter-4-rclpy-python-client.md
│   └── chapter-5-urdf-robot-description.md
│
├── module-2-simulation/                        # Module 2: The Digital Twin
│   ├── chapter-1-gazebo-physics-basics.md
│   ├── chapter-2-lidar-integration.md
│   ├── chapter-3-imu-integration.md
│   ├── chapter-4-depth-camera.md
│   └── chapter-5-unity-environment.md
│
├── module-3-isaac/                             # Module 3: The AI-Robot Brain
│   ├── chapter-1-isaac-sim-intro.md
│   ├── chapter-2-vslam-mapping.md
│   ├── chapter-3-nav2-navigation.md
│   ├── chapter-4-sim-to-real-principles.md
│   └── chapter-5-domain-randomization.md
│
└── module-4-vla/                               # Module 4: Vision-Language-Action
    ├── chapter-1-whisper-voice.md
    ├── chapter-2-llm-task-planning.md
    ├── chapter-3-nlp-to-ros-actions.md
    ├── chapter-4-multimodal-control.md
    └── chapter-5-capstone-integration.md

website/
├── docusaurus.config.ts     # Existing (from 001-landing-page)
├── sidebars.ts              # TO UPDATE: Add textbook sidebar structure
├── src/                     # Existing (from 001-landing-page)
│   ├── pages/index.tsx      # Landing page (already implemented)
│   ├── components/          # Hero, StatsSection (already implemented)
│   ├── css/custom.css       # Humaride theme (already configured)
│   └── styles/              # Theme colors (already configured)
└── static/                  # Images, assets (if needed for textbook)
```

### Code Validation Environment (Docker - for testing Python examples)

```text
.docker/
└── textbook-validator/
    ├── Dockerfile                   # Ubuntu 22.04 + ROS 2 Humble + Python 3.10
    ├── validate-chapter.sh          # Script to run all code examples in a chapter
    └── requirements.txt             # Python dependencies (rclpy, etc.)
```

**Structure Decision**: This is a documentation/content creation project. The primary deliverable is 21 markdown files in `website/docs/` organized into module directories. Unlike traditional software, there is no `src/` directory with application code. Python code exists only as embedded examples within markdown chapters (code blocks), not as standalone .py files. Validation is performed via Docker container that executes code blocks extracted from chapters.

## Complexity Tracking

**No violations to justify** - All applicable constitution principles (I-IV) pass without exceptions.
