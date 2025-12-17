# ADR-0001: Technology Stack Selection for Physical AI Robotics Textbook

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** Physical AI Robotics Book
- **Context:** Need to select a technology stack for building a comprehensive textbook website on Physical AI Robotics that will be deployed to GitHub Pages. The solution must support technical documentation with good search, responsive design, accessibility compliance, and performance optimization.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Framework: Docusaurus v3 (with React)
- Runtime: Node.js 18+
- Deployment: GitHub Pages
- Testing: Jest and Cypress
- Accessibility: WCAG 2.1 AA compliance
- Performance: <2s page load time, <1MB per page, 95%+ browser compatibility

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Docusaurus provides excellent built-in features for documentation websites (search, versioning, responsive design)
- Strong community support and extensive documentation for technical content
- Seamless integration with GitHub Pages for deployment
- Built-in accessibility features that support WCAG 2.1 AA compliance
- Optimized for technical documentation with support for code blocks, diagrams, and mathematical notation
- Static site generation provides excellent performance and security
- SEO-friendly with proper meta tags and structured content

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Learning curve for Docusaurus-specific concepts and configuration
- Potential vendor lock-in to Docusaurus ecosystem
- May have limitations compared to fully custom React applications
- Need to adapt to Docusaurus' conventions rather than complete flexibility
- Dependency on Node.js and npm ecosystem for build process

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

1. **GitBook**: Rejected because it offers less customization than Docusaurus and is limited to specific styling options. GitBook also requires a proprietary hosting solution or has limited self-hosting capabilities.

2. **Sphinx**: Rejected because it's more complex to set up and primarily designed for Python documentation. Would require significant customization to support the JavaScript/TypeScript content needed for this project.

3. **MkDocs**: Good alternative but less feature-rich than Docusaurus for this use case. While suitable for documentation, it lacks some of the advanced features and React-based customization capabilities of Docusaurus.

4. **Custom React App**: Rejected because it would be more complex to maintain and lacks built-in documentation features like search, versioning, and sidebar navigation. Would require implementing many features that Docusaurus provides out-of-the-box.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: specs/001-physical-ai-robotics-book/spec.md
- Implementation Plan: specs/001-physical-ai-robotics-book/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/001-physical-ai-robotics-book/research.md <!-- link to eval notes/PHR showing graders and outcomes -->
