import React, { ReactNode, useEffect } from "react";
import { useHistory, useLocation } from "@docusaurus/router";
import { useAuth } from "../../contexts/AuthContext";
import styles from "./styles.module.css";

interface AuthGuardProps {
  children: ReactNode;
}

// Routes that don't require authentication
const PUBLIC_ROUTES = [
  "/",
  "/auth/login",
  "/auth/signup",
  "/auth/reset-password",
  "/ur/",
  "/ur/auth/login",
  "/ur/auth/signup",
  "/ur/auth/reset-password",
];

// Check if a path is public
function isPublicRoute(pathname: string): boolean {
  // Exact match for public routes
  if (PUBLIC_ROUTES.includes(pathname)) {
    return true;
  }

  // Check if it's a localized version of auth routes
  if (pathname.match(/^\/[a-z]{2}\/auth\/(login|signup|reset-password)$/)) {
    return true;
  }

  // Homepage with locale
  if (pathname.match(/^\/[a-z]{2}\/?$/)) {
    return true;
  }

  return false;
}

export default function AuthGuard({ children }: AuthGuardProps): React.ReactElement {
  const { user, loading } = useAuth();
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    if (!loading && !user && !isPublicRoute(location.pathname)) {
      // Redirect to login with return URL
      const returnUrl = encodeURIComponent(location.pathname + location.search);
      history.push(`/auth/login?returnUrl=${returnUrl}`);
    }
  }, [user, loading, location.pathname, history]);

  // Show loading spinner while checking auth
  if (loading) {
    return (
      <div className={styles.loadingContainer}>
        <div className={styles.spinner}></div>
        <p className={styles.loadingText}>Loading...</p>
      </div>
    );
  }

  // If not authenticated and not on a public route, show nothing (will redirect)
  if (!user && !isPublicRoute(location.pathname)) {
    return (
      <div className={styles.loadingContainer}>
        <div className={styles.spinner}></div>
        <p className={styles.loadingText}>Redirecting to login...</p>
      </div>
    );
  }

  // User is authenticated or on a public route
  return <>{children}</>;
}
