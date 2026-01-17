import React, { ReactNode } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

interface AuthGuardProps {
  children: ReactNode;
}

/**
 * AuthGuard - Currently disabled as Firebase is not configured.
 * All content is publicly accessible.
 *
 * To enable authentication:
 * 1. Configure Firebase in .env
 * 2. Uncomment the auth logic below
 */
export default function AuthGuard({
  children,
}: AuthGuardProps): React.ReactElement {
  // Firebase not configured - allow all access
  // All content (docs, labs, podcasts) is publicly accessible
  return <>{children}</>;
}
