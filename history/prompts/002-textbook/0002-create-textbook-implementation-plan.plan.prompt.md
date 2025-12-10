---
id: 0002
title: Create Textbook Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-textbook
branch: 002-textbook
user: user
command: /sp.plan
labels: ["textbook", "docusaurus", "ros2", "physical-ai", "robotics", "planning", "architecture"]
links:
  spec: specs/002-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-textbook/plan.md
  - specs/002-textbook/research.md
  - specs/002-textbook/data-model.md
  - specs/002-textbook/contracts/chapter-schema.md
  - specs/002-textbook/quickstart.md
  - CLAUDE.md (updated with textbook technologies)
tests:
  - Constitution check: 4/8 principles applicable, all passing
  - Principle I: Educational Quality First - PASS
  - Principle II: Structured Content Architecture - PASS
  - Principle III: Code Correctness & Testability - PASS
  - Principle IV: Visual Learning Through Diagrams - PASS
  - Research phase: 7 technical decisions documented
  - Design phase: 5 entities modeled with validation rules
---

## Prompt

```
User approved specification with "3: Good" rating.

Implicit command execution: /sp.plan

Continue implementation planning workflow per Spec-Kit Plus protocol:
- Phase 0: Research (technical decisions)
- Phase 1: Design & Contracts (data model, chapter schema, quickstart guide)
- Update agent context with technologies

Feature context:
- 21-chapter Docusaurus textbook on Physical AI & Humanoid Robotics
- 4 modules: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action
- Mandatory chapter structure: learning objectives, theory, code examples, exercises
- Python 3.10+ code must run on Ubuntu 22.04 with ROS 2 Humble
- 2000–3000 words per chapter (excluding code)
- Mermaid diagrams exclusively for visuals
- PEP 8 compliance enforced via black + flake8
```

## Response snapshot

Created comprehensive implementation plan with:

**Technical Context**:
- **Languages**: Python 3.10+ (code examples), Markdown (content), Mermaid (diagrams)
- **Primary Dependencies**: Docusaurus 3.x, ROS 2 Humble, Gazebo Fortress, Unity 2021.3 LTS, NVIDIA Isaac Sim, OpenAI Whisper, LLM access
- **Storage**: Git repository for markdown, GitHub Pages for hosting
- **Testing**: Docusaurus build validation, Docker code execution (Ubuntu 22.04 + ROS 2 Humble)
- **Performance Goals**: Build <5 min, page load <2s, code executes <30s, Module 1 completable in 10 hours
- **Scale**: 21 chapters (~50,000 words), 50+ code examples, 20+ Mermaid diagrams, 100+ exercises

**Constitution Check** (8 principles evaluated):
- **Applicable (4/8)**: Educational Quality (I), Structured Content (II), Code Correctness (III), Visual Learning (IV) - ALL PASS ✅
- **Partial/Inherited (2/8)**: Performance & Accessibility (V), Brand Consistency (VI) - from 001-landing-page ✅
- **Out of Scope (2/8)**: User-Centric Features (VII), Intelligent Chatbot (VIII) - separate features ✅
- **GATE RESULT**: ✅ PASS - Proceed to research and design

**Phase 0: Research** (7 Technical Decisions):
1. **Chapter Template Structure**: YAML frontmatter + 8 mandatory sections (learning objectives, prerequisites, introduction, theory, code examples, exercises, summary, next steps)
2. **Mermaid Diagram Patterns**: 4 types for robotics (flowchart for node communication, sequence for service calls, state for behaviors, class for URDF/data structures)
3. **Code Validation Strategy**: Docker-based (Ubuntu 22.04 + ROS 2 Humble container), extracts code blocks from markdown, validates PEP 8 compliance, executes with 30s timeout
4. **Docusaurus Sidebar Configuration**: Category-based with collapsible modules, intro + 4 module categories
5. **Content Writing Guidelines**: Domain-appropriate analogies (radio broadcast for topics, restaurant for services), progressive exercises (guided → semi-guided → open-ended)
6. **Prerequisites & Dependency Management**: Version-pinned (ROS 2 Humble, Python 3.10+, Ubuntu 22.04), verification commands included
7. **PEP 8 Compliance**: Black formatter (88 char line length) + Flake8 linter enforced on all Python code

**Phase 1: Design & Contracts** (5 Entities):
1. **Chapter Entity**: 10 attributes (title, sidebar_position, module_id, learning_objectives [3-5], prerequisites, content_sections, word_count [2000-3000], code_examples [1+], mermaid_diagrams [1+], exercises [2-3])
2. **Module Entity**: 6 attributes (module_id, title, description, chapter_count [always 5], chapters, sidebar_position_range)
3. **CodeExample Entity**: 7 attributes (title, language [always "python"], code [5-100 lines], expected_output, execution_time_seconds [<30], dependencies, pep8_compliant [always true])
4. **MermaidDiagram Entity**: 5 attributes (type [flowchart/sequence/state/class], title, mermaid_code, node_count [≤12], validates [always true])
5. **Exercise Entity**: 6 attributes (title, type [guided/semi-guided/open-ended/discussion], difficulty [beginner/intermediate/advanced], instructions, hints, estimated_time_minutes [10-60], solution_available)

**Validation Rules**:
- Chapter: 7 validation rules (frontmatter, sections, word count, learning objectives, code examples, Mermaid diagrams, exercises)
- Module: 4 validation rules (chapter count, sidebar positions, naming, completeness)
- CodeExample: 5 validation rules (executability, output match, timeout, PEP 8, dependencies)
- Cross-entity: 4 validation rules (prerequisite links, sidebar positions, module ordering, next chapter links)

**Contract**: `contracts/chapter-schema.md` with:
- Frontmatter schema (YAML with title [3-60 chars] and sidebar_position [1-21, unique])
- Section structure (8 mandatory sections with specific subsections)
- Content validation rules (word count, learning objective verbs, prerequisites format)
- Automated validation checklist (26 items for CI/CD)
- File naming convention (`module-X-theme/chapter-Y-topic.md`)
- Minimal valid chapter example (390 lines showing all required elements)

**Quickstart Guide**: `quickstart.md` with:
- Complete chapter template (copy-paste ready)
- Sample chapter: "ROS 2 Topics and Publishers" (demonstrates all 8 sections)
- 4 Mermaid diagram examples (flowchart, sequence, state, class)
- Code validation workflow (black, flake8, Docker execution)
- Docusaurus sidebar configuration (TypeScript snippet)
- File naming convention with valid/invalid examples
- Quality checklist (24 items across content, structure, code, technical accuracy, accessibility)
- Common pitfalls with ❌ bad / ✅ good examples

**Project Structure**:
```
website/docs/
├── intro.md                    # Introduction (P1 MVP)
├── module-1-ros2/              # 5 chapters
├── module-2-simulation/        # 5 chapters
├── module-3-isaac/             # 5 chapters
└── module-4-vla/               # 5 chapters

.docker/textbook-validator/     # Ubuntu 22.04 + ROS 2 Humble validation
specs/002-textbook/             # Planning artifacts
```

## Outcome

- ✅ Impact: Created production-ready implementation plan for 21-chapter textbook. All technical decisions finalized (no NEEDS CLARIFICATION items). Constitution check passed all gates (4 applicable, 2 partial, 2 out of scope). Ready for `/sp.tasks` command to generate granular task list.
- 🧪 Tests: Constitution check passed (4/4 applicable principles). Research phase validated 7 decisions. Design phase validated 5 entities with complete schemas. Cross-entity validation rules defined (prerequisite links, sidebar positions, module ordering).
- 📁 Files: Created `plan.md` (213 lines, technical context + constitution check), `research.md` (462 lines, 7 decisions), `data-model.md` (348 lines, 5 entities), `contracts/chapter-schema.md` (346 lines, validation contract), `quickstart.md` (comprehensive guide with templates and examples). Updated `CLAUDE.md` with textbook technologies.
- 🔁 Next prompts: Run `/sp.tasks` to break down implementation into granular tasks. Consider `/sp.clarify` if user wants to refine any technical decisions (e.g., alternative simulation platforms, different LLM providers, additional module topics).
- 🧠 Reflection: Planning phase comprehensive and thorough. 7 research decisions cover all technical aspects (chapter structure, diagrams, validation, sidebar, content guidelines, prerequisites, code formatting). 5 entities fully modeled with validation rules. Quickstart guide provides concrete examples (sample chapter, Mermaid patterns, Docker setup). Trade-off: High prescriptiveness (88 char line length, 2000-3000 words, exactly 5 chapters per module) may limit flexibility during implementation. However, this aligns with constitution principle II (Structured Content Architecture) which emphasizes consistency. Next phase (/sp.tasks) should break down into small, incremental tasks to manage complexity (21 chapters is substantial scope).

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow executed smoothly. All constitution principles evaluated correctly. Research decisions comprehensive. Entity modeling complete. No missing dependencies or unresolved questions.
- Graders run and results (PASS/FAIL): Constitution check = PASS (4/4 applicable). Research phase = PASS (7 decisions, no NEEDS CLARIFICATION). Design phase = PASS (5 entities, complete schemas). Validation rules = PASS (26 checklist items defined).
- Prompt variant (if applicable): Standard `/sp.plan` workflow. No variants tested.
- Next experiment (smallest change to try): For future planning phases, consider creating a "sample artifact" (like the quickstart chapter example) during Phase 0 research instead of Phase 1 design. This would provide concrete validation of technical decisions earlier in the process. For example, writing a minimal chapter stub during research would validate that the 8-section template actually works before committing to it in design phase. This "test-driven planning" approach could surface issues earlier.
