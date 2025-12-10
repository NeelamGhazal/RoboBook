# Physical AI & Humanoid Robotics Textbook Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 1.3.0 → 1.4.0
Rationale: MINOR version bump - Added theme toggle functionality, comprehensive navbar specifications, precise typography rules, detailed hero section requirements, strict file modification restrictions, and performance standards

Modified Principles:
- VIII. Mandatory Page Structure: Added precise navbar dimensions (72px height, specific icon sizes, spacing requirements)
- VIII. Navbar: Added theme toggle requirement with light theme palette
- VI. Brand Consistency: Clarified Orbitron usage limited to landing page only (not docs/blog)
- XIII. Landing Page: Added detailed spacing requirements, exact font sizes, precise image dimensions

Added Sections:
- XX. Theme Toggle (Mandatory UI Control): Light/Dark theme toggle with React state, 200ms transitions, WCAG AA compliance
- File Modification Restrictions: Explicit allowed/forbidden files list
- Typography Rules: Precise font size specifications for all landing page elements
- Performance Rules: Image optimization, no layout shift requirements
- Non-Negotiable Restrictions: No new pages, backend, translation system, routing changes

Removed Sections:
- None (all additions are expansions of existing principles)

Templates Status:
- ✅ .specify/templates/plan-template.md - Constitution Check section aligns
- ✅ .specify/templates/spec-template.md - Requirements structure aligns
- ✅ .specify/templates/tasks-template.md - Task organization aligns

Follow-up TODOs:
- Implement theme toggle in navbar with React state
- Add light theme CSS variables and styling
- Optimize robot and card images (webp format)
- Ensure navbar matches exact 72px height specification
- Test theme toggle transition (200ms ease-in-out)
- Verify WCAG AA contrast compliance in both themes

Last Updated: 2025-12-10
-->

## Core Principles

### I. Educational Quality First (NON-NEGOTIABLE)

**RULE**: Every chapter MUST deliver measurable learning value with clear objectives, tested code, and exercises.

**Requirements**:
- 3–5 learning objectives per chapter (measurable verbs: "implement", "explain", "configure")
- Theory with practical analogies (no abstract jargon without context)
- All code MUST be copy-paste runnable on Ubuntu 22.04 with Python 3.10+
- 2000–3000 words per chapter (excluding code blocks)
- Exercises that require active engagement (discussion OR implementation)
- Prerequisites stated at chapter top

**Rationale**: This textbook represents Panaversity's commitment to AI-native education. Poor quality content undermines the entire mission and user trust.

### II. Structured Content Architecture

**RULE**: 21 chapters organized across 4 modules with strict template adherence.

**Structure**:
- **Introduction**: Foundations, Embodied Intelligence, Course Roadmap
- **Module 1 (ROS 2)**: 5 chapters on nodes, topics, services, rclpy, URDF
- **Module 2 (Digital Twin)**: 5 chapters on Gazebo, Unity, physics simulation, sensors
- **Module 3 (NVIDIA Isaac)**: 5 chapters on Isaac Sim, Isaac ROS, VSLAM, Nav2, Sim-to-Real
- **Module 4 (VLA)**: 5 chapters on Whisper, LLMs, ROS actions, autonomous humanoid capstone

**Mandatory Chapter Template**:
1. Learning Objectives
2. Theory
3. Code Examples (with input + expected output)
4. Exercises
5. Prerequisites (at top)

**Rationale**: Consistent structure enables progressive learning and predictable user experience. Random organization confuses learners.

### III. Code Correctness & Testability

**RULE**: All code examples MUST be verified, tested, and include both input and expected output.

**Standards**:
- Python 3.10+ with PEP 8 compliance
- ROS 2 Humble or later
- Tested on Ubuntu 22.04 LTS
- Code blocks show complete setup (imports, initialization, execution)
- Output included as comments or separate block
- No pseudocode or "fill in the blanks" unless explicitly marked as exercise

**Rationale**: Students lose trust and learning momentum when code fails. Copy-paste runnability is the minimum bar for technical education.

### IV. Visual Learning Through Diagrams

**RULE**: Use Mermaid diagrams exclusively for system visualizations. No external images except where technically necessary.

**Requirements**:
- System architectures as flowcharts or sequence diagrams
- Data flows as directed graphs
- State machines for robot behaviors
- ROS 2 node communication graphs
- No decorative images (every visual must teach)

**Rationale**: Mermaid ensures version control, maintainability, and consistent rendering. External images create deployment dependencies and accessibility issues.

### V. Performance & Accessibility

**RULE**: Landing page MUST load <3 seconds, achieve Lighthouse >90, and meet WCAG 2.1 AA standards.

**Standards**:
- Lighthouse Performance: >90
- Lighthouse Accessibility: >90
- Mobile responsive (tested on iPhone 12, Samsung Galaxy S21)
- Keyboard navigation for all interactive elements
- Proper heading hierarchy (h1 → h2 → h3)
- Alt text for functional images
- Color contrast ratios ≥4.5:1 for text

**Rationale**: Poor performance and accessibility exclude learners with slow connections or disabilities. Non-negotiable for educational equity.

### VI. File Modification Restrictions (NON-NEGOTIABLE)

**RULE**: Only specified files may be modified. All other files are strictly off-limits.

**Allowed Modifications**:
- `src/pages/index.tsx` (landing page only)
- `src/components/Hero/*` (hero section components)
- `src/components/FeatureCards/*` (feature card components)
- `src/css/custom.css` (landing page styling only)

**Forbidden Modifications**:
- Routing configuration
- Docs/blog content or styling
- Docusaurus config (except navbar items as specified)
- Animation code files
- Any files outside the allowed scope
- Backend logic
- Translation systems
- Chatbot implementation

**Rationale**: Strict file modification boundaries prevent scope creep, maintain system stability, and ensure changes remain focused on landing page improvements only.

### VII. Brand Consistency (RoboBook - Ocean Sapphire)

**RULE**: All UI elements MUST use the Ocean Sapphire color palette and design system.

**Color Palette (Ocean Sapphire Theme)**:
- Primary Deep Blue: `#001529`
- Mid Ocean Blue: `#002140`
- Light Ocean Blue: `#003a6d`
- Accent Cyan: `#0096ff`
- Soft Cyan: `#00d4ff`
- Light Cyan: `#80c2ff`
- Pale Blue Text: `#b8d4ff`
- Card Background (Dark): `rgba(0, 50, 100, 0.3)`
- Border Glow: `rgba(0, 150, 255, 0.3)`
- Shimmer Effect: `rgba(0, 150, 255, 0.1)`

**Design Standards (Ocean Sapphire Theme)**:
- Headings: Orbitron (elegant, professional tech font)
- Body: 16–18px, line-height 2.0, color `#b8d4ff`
- Code: Fira Code monospace (preserved from original)
- Cards: Glassmorphism with `backdrop-filter: blur(10px)`
- Borders: 1px solid `rgba(0, 150, 255, 0.3)` with subtle glow
- Hover Effects:
  - Buttons: `box-shadow: 0 0 20px rgba(0, 150, 255, 0.5)`
  - Cards: Shimmer animation (3s infinite)
- Corner Radius: 12px for cards (soft, modern)
- Background: Gradient `linear-gradient(180deg, #001529 0%, #002140 50%, #003a6d 100%)`
- Ambient Glow: Radial gradients at 20% and 80% viewport positions

**Typography Hierarchy (Landing Page Only)**:
- Hero Heading: 4.5rem (72px), Orbitron, 700 weight, line-height 1.2
- Hero Paragraph: 1.3rem (20px), Georgia serif or system serif
- Button Text: 1.2rem (18px), Orbitron, 700 weight
- Section Heading: 2.8rem (45px), Georgia serif
- Card Titles: 1.5rem (24px), Orbitron, 700 weight
- Card Text: 1rem (16px), Georgia serif

**Typography Restrictions**:
- Orbitron ONLY for: Navbar, landing page headings, landing page CTAs
- Orbitron NOT allowed in: Docs content, Blog, Sidebar, Chapters
- Docs/blog maintain default fonts (no changes allowed)

**Rationale**: Ocean Sapphire provides elegant readability with professional aesthetic. Deep blues reduce eye strain during long reading sessions while maintaining visual sophistication. Glassmorphism creates modern premium feel aligned with AI/robotics content. Orbitron font adds tech-forward, robotic aesthetic appropriate for the RoboBook brand, but is limited to landing page to maintain readability in long-form content.

### VII. UI/UX Design System

**RULE**: All UI elements MUST adhere to the Ocean Sapphire design system specifications.

**Color Palette (Ocean Sapphire)**:
- Primary Dark: `#001529`
- Primary Mid: `#002140`
- Accent Cyan: `#0096ff`
- Bright Cyan: `#00d4ff`
- Light Cyan: `#80c2ff`
- Pale Text: `#b8d4ff`

**Typography**: Georgia serif for all headings; body in readable sans-serif

**Layout**: Mobile-first, responsive grid system

**Accessibility**: WCAG AA compliant contrast; all animations respect `prefers-reduced-motion`

### VIII. Mandatory Page Structure

**RULE**: All pages MUST follow the specified structure with navbar, hero section, features section, and feature cards.

#### Navbar (Fixed Top, Backdrop Blur)

**Dimensions (MUST match exactly)**:
- Height: 72px fixed
- Logo icon: 40×40px
- Logo container: 200×60px max width
- Logo text: "RoboBook" (Orbitron, 22px, 700 weight)
- Menu item font-size: 17px
- Menu item gap: 32px horizontal spacing
- Right controls (language + theme + GitHub): 28px icons, 24px spacing between

**Left Side**:
- Logo: Robotic minimal icon (40×40px) + "RoboBook" text
- Interaction: Logo hover → subtle cyan glow (`box-shadow: 0 0 15px rgba(0, 150, 255, 0.4)`)

**Right Side (in order)**:
- Navigation items: `Textbook | Blog`
- Language Toggle: `EN | UR` (visual only, before GitHub)
- Theme Toggle: Light/Dark mode switch (last item, see Section XX)
- GitHub Icon: Link to repository

**Rationale**: Fixed navbar dimensions ensure visual consistency and prevent layout shifts. Precise spacing creates professional appearance matching reference design.

#### Hero Section (Responsive Grid)

**Layout (MUST match reference exactly)**:
- Desktop (≥1024px): 50% Left / 50% Right
- Tablet (768–1023px): 60% Left / 40% Right
- Mobile (<768px): Vertical stack (text → image)

**Spacing**:
- Top padding: 100px
- Side padding: minimum 24px (px-6)
- Gap heading → paragraph: 20px
- Gap paragraph → button: 28px

**Left Side (Text)**:
- Heading: "Physical AI & Humanoid Robotics"
  - Font: Orbitron, 4.5rem (72px), 700 weight
  - Line-height: 1.2
  - Max-width: 580px
  - Color: white with blue glow
- Paragraph: "Learn to control physical androids using ROS 2 and Isaac Sim—even if you only have a laptop"
  - Font: Georgia serif or system serif
  - Size: 1.3rem (20px)
  - Max-width: 520px
  - Color: `#80c2ff`
- Button: "Start Learning"
  - Size: 200px × 60px
  - Font: 1.2rem (18px), Orbitron, 700 weight
  - Border: 2px solid `#0096ff`
  - Hover: Cyan glow effect

**Right Side (Robot Image)**:
- Professional 3D half-body robot image, high resolution
- Image sizing:
  - Width: 520–580px
  - Height: auto (maintain proportions)
  - Mobile: width 100%, centered
- Format: WebP preferred for optimization
- Must NOT distort proportions

#### Features Section

**Layout**:
- Max-width: 1200px, centered
- Section vertical padding: 80px
- Card gap: 24px on desktop, 16px mobile

**Heading**:
- Text: "What's Inside This Book?"
- Font: Georgia serif
- Size: 2.8rem (45px)
- Margin-bottom: 16px
- Centered

**Description**:
- Text: "This book guides you step-by-step through building a complete Physical AI project—from writing a spec-driven book to embedding an intelligent RAG chatbot, personalizing content, and integrating humanoid robotics simulations."
- Max-width: 900px
- Font-size: 1.1rem
- Center aligned
- Margin-bottom: 48px

#### Feature Cards (3 Items, Stacked Vertically)

**Card Dimensions**:
- 3 cards stacked vertically with gaps
- Min height: 220px per card
- Border-radius: 12px
- Padding: 24px
- Aspect ratio: 3:2

**Card Layout**:
- LEFT: Image (40% width)
- RIGHT: Text (60% width)

**Card Images**:
- Aspect ratio: 3:2
- Robotics/tech themed
- Width: 40% of card, minimum 180px
- Height: auto
- Format: WebP preferred

**Card Styling**:
- Glassmorphism: `background: rgba(0, 50, 100, 0.3)`, `backdrop-filter: blur(15px)`
- Border: `1px solid rgba(0, 150, 255, 0.3)`
- Border-radius: 12px
- Hover: Shimmer animation (cyan gradient sweep, 3s duration)

**Card Content**:
1. "AI/Spec-Driven Book Creation" (title: 1.5rem, Orbitron, 700) – "Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus." (text: 1rem, Georgia)
2. "Integrated RAG Chatbot" (title: 1.5rem, Orbitron, 700) – "Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres." (text: 1rem, Georgia)
3. "Personalization & Translations" (title: 1.5rem, Orbitron, 700) – "Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter." (text: 1rem, Georgia)

### IX. Technical Constraints

**RULE**: Implementation MUST follow specified technical constraints for performance and functionality.

- **Language Toggle**: EN/UR in React state; Urdu = RTL alignment
- **No localStorage**: All state in React only
- **Performance**: Canvas optimized for 60fps; page load <2s
- **Framework**: Docusaurus + React + Tailwind CSS
- **Icons**: lucide-react only

### X. Quality Standards

**RULE**: All visual elements MUST meet specified quality standards.

- **Color Accuracy**: Strict adherence to Ocean Sapphire hex values
- **Responsiveness**: Breakpoints at 1024px (desktop), 768px (tablet), 640px (mobile)
- **Animation Smoothness**: All hover effects = 300ms; shimmer = 3s infinite
- **Visual Fidelity**: Robot wireframe must be clearly humanoid with joints, fingers, and antenna

### XI. User-Centric Features (Personalization & Localization)

**RULE**: Implement Better-Auth with user profiling and per-chapter personalization/translation.

**Required Features**:
- **Authentication**: Email/password via Better-Auth
- **User Survey** (at signup):
  - Software skill level (Beginner → Expert)
  - Hardware skill level (None → Expert)
  - Robotics experience (None → Real robots)
- **Personalization Button** (top of every chapter):
  - Simplified (for beginners)
  - Standard (default)
  - Advanced (for experts)
  - Uses user profile to suggest default level
- **Urdu Translation Button** (top of every chapter):
  - Translates to Roman Urdu
  - Preserves code blocks (no translation)
  - Supports RTL layout for Urdu text

**Rationale**: Personalization respects diverse learner backgrounds. Urdu localization expands accessibility to Pakistan's primary language audience.

### XII. Intelligent Chatbot (RAG-Based)

**RULE**: Embed a RAG chatbot that answers text-selection queries with <2s response time and source citations.

**Architecture**:
- **Frontend**: Text selection triggers query
- **Backend**: FastAPI endpoint
- **Vector DB**: Qdrant Cloud (free tier)
- **Database**: Neon Postgres (chat history, user preferences)
- **AI**: OpenAI API (gpt-4o-mini) via ChatKit SDK
- **Embedding**: Text chunks from all 21 chapters

**Requirements**:
- Response time: <2 seconds
- Source citations: Chapter name + section
- Context window: 3 paragraphs around selection
- Error handling: Graceful fallback if API fails

**Rationale**: RAG chatbot provides just-in-time learning support without leaving the reading flow. Reduces friction in understanding complex topics.

## Content Standards

### Chapter Word Count
- **Target**: 2000–3000 words (excluding code)
- **Rationale**: Enough depth for mastery, short enough for single-session consumption

### Code Standards
- **Language**: Python 3.10+, PEP 8 compliant
- **Framework**: ROS 2 Humble or later
- **OS**: Ubuntu 22.04 LTS
- **Output**: Always show expected output (as comments or separate block)
- **Testing**: Every code example must be verified before publication

### Exercises
- **Types**: Discussion questions OR implementation challenges
- **Quantity**: 2–3 per chapter
- **Clarity**: Unambiguous instructions, clear success criteria

### Prerequisites
- **Location**: Top of every chapter
- **Format**: Bullet list of prior knowledge/chapters required
- **Examples**: "Completed Chapter 3: ROS 2 Topics", "Familiar with Python classes"

## Technical Architecture

### Frontend Stack
- **Framework**: Docusaurus 3.x
- **Styling**: Tailwind CSS + CSS Modules
- **Animations**: Framer Motion (UI), Three.js (landing page only)
- **Deployment**: GitHub Pages

### Backend Stack
- **API**: FastAPI (chatbot + personalization endpoints)
- **Database**: Neon Postgres (users, preferences, chat history)
- **Vector DB**: Qdrant Cloud (chapter embeddings)
- **AI**: OpenAI API (gpt-4o-mini), ChatKit SDK
- **Deployment**: Render/Vercel

### Authentication
- **Library**: Better-Auth
- **Method**: Email/password
- **Session**: JWT with HTTP-only cookies
- **Survey**: User background questionnaire at signup

### XIII. Landing Page (Custom Docusaurus Page at `/`)
- **Layout**: 50/50 split (desktop), stacked (mobile)
- **Background**:
  - Gradient: `linear-gradient(180deg, #001529 0%, #002140 50%, #003a6d 100%)`
  - Ambient Effects: Radial gradients at 20%/30% and 80%/70% positions with `rgba(0, 150, 255, 0.1)`
- **Left Panel**: Text content with Orbitron typography
  - Heading: "Physical AI & Humanoid Robotics" (Orbitron, 4.5rem, line-height 1.2)
  - Subheading: "Learn to control physical androids using ROS 2 and Isaac Sim—even if you only have a laptop" (Orbitron italic, 1.3rem)
  - "Start Learning" Button:
    - Size: 200×60px
    - Border: `1px solid #0096ff`
    - Hover: cyan glow + slight scale
    - Rounded corners consistent with theme
- **Right Panel**: Static robot/tech illustration image (replaces animation)
  - Responsive: must shrink correctly on all devices
- **Features Section**: Insert below Hero Section, before any other content
  - Heading: "What's Inside This Book?" (Orbitron, 2.8rem, centered, mb-6)
  - Description: "This book guides you step-by-step through building a complete Physical AI project—from writing a spec-driven book to embedding an intelligent RAG chatbot, personalizing content, and integrating humanoid robotics simulations." (centered, max-width 900px, 1.1rem, mb-16)
- **Feature Cards**: 3 glassmorphism cards in flex row layout
  - Layout: Flex row with left image (40%) and right text (60%)
  - Aspect ratio: 3:2
  - Background: `rgba(0, 50, 100, 0.3)` with `backdrop-filter: blur(10px)`
  - Border: `1px solid rgba(0, 150, 255, 0.3)` with rounded 12px
  - Hover: Shimmer animation (gradient sweep from left to right, 3s infinite)
  - Gap: 24px between cards
  - Mobile: Stack vertically (image → text)
  - Content:
    1. "AI/Spec-Driven Book Creation" – "Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus." (AI/tech illustration image)
    2. "Integrated RAG Chatbot" – "Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres." (Chatbot/AI agent illustration image)
    3. "Personalization & Translations" – "Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter." (Globe/translation-themed illustration image)

### XIV. Book Reader Page (`/docs/*`)
- **Background**: Dark gradient `#0d1117` with subtle blue ambient glow
- **TOC Sidebar**:
  - Collapsible, hierarchical (modules → chapters)
  - Active chapter: Border-left 3px solid `#0096ff` with glow
  - Hover: Background `rgba(0, 150, 255, 0.1)`
- **Typography**:
  - Headings: Georgia serif, white with subtle letter-spacing
  - Body: 16–18px Georgia, line-height 2.0, color `#b8d4ff`
  - Code: Fira Code monospace (preserved)
- **Chapter Cards**:
  - Background: `rgba(0, 50, 100, 0.3)` with glassmorphism
  - Border: `1px solid rgba(0, 150, 255, 0.3)`, rounded 12px
  - Shimmer effect on hover (3s infinite gradient sweep)
  - Shadow: `0 10px 40px rgba(0, 0, 0, 0.5)`
- **Chapter Badges**:
  - Background: `rgba(0, 150, 255, 0.2)`
  - Border: `1px solid #0096ff`, rounded 30px
  - Text: `#00d4ff`, uppercase, letter-spacing 2px
  - Glow: `0 0 15px rgba(0, 150, 255, 0.3)`
- **Highlight Boxes** (Key Insights):
  - Background: `rgba(0, 100, 200, 0.15)`
  - Border-left: 4px solid `#0096ff`
  - Icon: 💎 (cyan color)
  - Padding: 30px, margin 40px 0
- **Animations**:
  - Page fade-in with slight upward slide (0.6s ease)
  - Scroll-triggered reveals for content blocks
  - Active chapter highlight with cyan glow
  - Smooth transitions (0.3s ease-out)
- **Code Blocks**:
  - Dark theme (preserved)
  - Copy button with cyan hover effect
  - Syntax highlighting maintained

## XV. UI/UX Requirements

### Responsiveness
- **Desktop**: 1920×1080 (primary), 1366×768 (tested)
- **Mobile**: iPhone 12 (390×844), Samsung Galaxy S21 (360×800)
- **Breakpoints**: 640px, 768px, 1024px, 1280px (Tailwind defaults)

### Performance Targets
- **Landing Page Load**: <3 seconds
- **Lighthouse Performance**: >90
- **Lighthouse Accessibility**: >90
- **First Contentful Paint**: <1.5s
- **Time to Interactive**: <3s

**Ocean Sapphire Performance Notes**:
- Glassmorphism `backdrop-filter` requires modern browsers (95%+ support)
- Shimmer animations use `transform` (GPU-accelerated)
- Gradient backgrounds are static (no performance penalty)
- All effects tested on mid-range devices (60fps maintained)

### Animations (Ocean Sapphire Theme)
- **Landing Page**:
  - Stats cards: Shimmer on hover (gradient sweep, 3s infinite)
  - Button hover: Glow expansion (0.3s ease)
- **Book Reader**:
  - Page transitions: Fade-in with upward slide (0.6s ease)
  - Scroll reveals: Content blocks appear with opacity + transform
  - Active chapter: Cyan glow pulse (subtle, 2s infinite)
  - Card hover: Enhanced shimmer effect
- **Interactive Elements**:
  - Buttons: Background shift + glow expansion
  - Links: Color change to `#00d4ff` with underline slide-in
  - Badges: Scale 1.05 on hover with glow intensification
- **Performance**: All animations GPU-accelerated, 60fps maintained

### Ocean Sapphire Design Philosophy

**Visual Identity**:
- **Sophistication**: Deep ocean blues evoke professionalism and depth
- **Readability**: High-contrast pale blue text (`#b8d4ff`) on dark backgrounds ensures comfortable long-form reading
- **Modern Premium**: Glassmorphism with shimmer effects creates contemporary, high-end aesthetic
- **Calmness**: Gradient backgrounds provide visual interest without distraction

**Color Psychology**:
- Deep blues: Trust, intelligence, stability (perfect for technical education)
- Cyan accents: Innovation, technology, clarity
- Subtle glows: Premium feel without overwhelming brightness

**Accessibility Compliance**:
- Text contrast: `#b8d4ff` on `#001529` = 8.2:1 (exceeds WCAG AAA)
- Headings contrast: White on dark blue = 14.5:1 (maximum readability)
- Accent contrast: `#00d4ff` provides 7.8:1 for interactive elements
- Glow effects: Decorative only, never rely on them for critical information

**Technical Rationale**:
- Glassmorphism `backdrop-filter`: Creates depth without heavy graphics
- Shimmer animations: Engage users without distraction (3s duration = subtle)
- Orbitron font: Tech-forward, robotic aesthetic appropriate for AI/robotics content
- Gradient backgrounds: Visual richness with minimal performance cost

### Accessibility (WCAG 2.1 AA)
- **Keyboard Navigation**: All interactive elements
- **Screen Readers**: Proper ARIA labels
- **Color Contrast**: Text ≥4.5:1, UI elements ≥3:1
- **Focus Indicators**: Visible on all focusable elements

**Ocean Sapphire Accessibility**:
- All text colors tested with contrast checkers
- Shimmer effects are decorative (not essential for understanding)
- Focus indicators use cyan glow (highly visible on dark backgrounds)
- Screen readers ignore decorative gradient backgrounds
- Reduced motion: Shimmer animations respect `prefers-reduced-motion`

## XVI. Workflow & Process

### Development Workflow (MANDATORY)
All features MUST follow Spec-Kit Plus workflow:

```
/sp.constitution → /sp.specify → /sp.clarify → /sp.plan → /sp.tasks → /sp.implement
```

### Per-Feature Process
1. **Specify** (`/sp.specify`): Define feature requirements with user stories
2. **Clarify** (`/sp.clarify`): Resolve ambiguities (2–5 targeted questions)
3. **Plan** (`/sp.plan`): Architecture decisions, ADR suggestions
4. **Tasks** (`/sp.tasks`): Actionable, dependency-ordered task list
5. **Implement** (`/sp.implement`): Execute tasks with PHR documentation

### Prompt History Records (PHR)
- **Trigger**: Every user interaction (except `/sp.phr` itself)
- **Location**: `history/prompts/` (auto-routed by stage)
- **Content**: Full user input (verbatim), concise assistant response, metadata (stage, feature, files, tests)

### Architecture Decision Records (ADR)
- **Trigger**: Significant architectural decisions (3-part test: impact, alternatives, scope)
- **Suggestion Format**: "📋 Architectural decision detected: [brief]. Document? Run `/sp.adr [title]`"
- **Consent Required**: Never auto-create ADRs
- **Location**: `history/adr/`

### Commit Standards
- **Format**: Conventional Commits (feat, fix, docs, chore, refactor, test)
- **Examples**:
  - `feat: add chapter 5 on ROS 2 services`
  - `fix: correct code output in chapter 3`
  - `docs: update constitution with UI/UX standards`
- **Frequency**: After each task or logical group

## XVII. Governance

### Constitution Authority
This constitution supersedes all other development practices. In case of conflict, constitution rules apply.

### Amendment Process
1. **Proposal**: Document proposed changes with rationale
2. **Review**: Team review (minimum 2 reviewers)
3. **Approval**: Unanimous approval required for MAJOR changes
4. **Version Bump**:
   - **MAJOR**: Backward-incompatible governance changes, principle removals
   - **MINOR**: New principles, materially expanded guidance
   - **PATCH**: Clarifications, wording improvements, typo fixes
5. **Migration**: Update dependent templates and documentation
6. **Sync Report**: HTML comment at top of constitution file

### Compliance Review
- **Frequency**: Every feature spec (`/sp.specify`)
- **Gate**: Constitution Check in `plan.md` (must pass before Phase 0 research)
- **Violations**: Must be justified in Complexity Tracking table

### Complexity Justification
Any violation of constitution principles MUST be documented:
- **Violation**: Which principle/rule
- **Why Needed**: Specific technical reason
- **Simpler Alternative Rejected**: Why standard approach insufficient

### Submission Requirements (Hackathon)
- **Deadline**: Nov 30, 2025, 6:00 PM PKT
- **Deliverables**:
  - Public GitHub repo
  - Live GitHub Pages URL
  - <90s demo video (YouTube/Drive)
  - WhatsApp number
- **Submission Form**: https://forms.gle/CQsSEGM3GeCrL43c8

### Demo Video Requirements
- **Duration**: <90 seconds
- **Content**:
  - Landing page animation (Three.js cube, stats cards)
  - Book navigation (TOC, chapter transitions)
  - RAG chatbot (text selection query, <2s response)
  - Authentication + user survey
  - Personalization button (Simplified/Standard/Advanced)
  - Urdu translation button (Roman Urdu, RTL layout)

## XX. Theme Toggle (Mandatory UI Control)

**RULE**: The landing page MUST include a Light/Dark theme toggle located in the navbar.

**Requirements**:
- Default theme: Dark (Ocean Sapphire)
- Toggle switches between Dark Theme and Light Theme
- Only colors change — layout, spacing, typography, animations remain identical
- Toggle controlled via React state (no localStorage)
- Transition animation: 200ms ease-in-out
- Must remain WCAG AA contrast-compliant in both themes

**Light Theme Palette**:
- Background: `#f3f8ff`
- Text: `#001529`
- Accent Cyan: `#0096ff` (unchanged)
- Borders: `rgba(0, 150, 255, 0.3)` (unchanged)
- Cards: `rgba(255, 255, 255, 0.6)` with blur

**Dark Theme Palette** (existing Ocean Sapphire):
- Background: `#001529`
- Text: `#b8d4ff`
- Accent Cyan: `#0096ff`
- Borders: `rgba(0, 150, 255, 0.3)`
- Cards: `rgba(0, 50, 100, 0.3)` with blur

**Placement**:
- Navbar → Right side, after EN/UR language toggle, before GitHub icon
- Icon size: 28px
- Spacing: 24px from adjacent controls

**Rationale**: Provides user-controlled accessibility and style preference while maintaining brand identity. Light mode improves readability in bright environments. React state ensures instant switching without page reload.

## XXI. Performance Standards

**RULE**: All images and assets MUST be optimized for web performance.

**Image Optimization**:
- Robot image: WebP format preferred, max 200KB
- Card images: WebP format, max 50KB each
- All images: Lazy loading enabled
- No layout shift on image load

**Performance Targets**:
- No layout shift (CLS = 0)
- No additional heavy JavaScript
- Theme toggle transition: 200ms maximum
- All animations respect `prefers-reduced-motion`

**Rationale**: Optimized assets ensure fast load times and smooth user experience. WebP format provides superior compression while maintaining visual quality.

## XXII. Non-Negotiable Restrictions

**RULE**: The following modifications are strictly forbidden:

**Forbidden Actions**:
- Creating new pages or routes
- Adding backend logic or API endpoints
- Implementing translation systems
- Adding chatbot functionality
- Modifying routing configuration
- Changing docs/blog fonts or styling
- Adding heavy JavaScript libraries
- Modifying files outside allowed scope

**Rationale**: These restrictions maintain project focus on landing page improvements only, prevent scope creep, and preserve system stability. All functionality must be frontend-only with React state management.

**Version**: 1.4.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-10
